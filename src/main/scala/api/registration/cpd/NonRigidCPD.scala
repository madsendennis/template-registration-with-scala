package api.registration.cpd

import api.registration.utils.PointSequenceConverter
import breeze.linalg.{Axis, DenseMatrix, DenseVector, diag, sum}
import scalismo.common.Vectorizer
import scalismo.geometry.{NDSpace, Point}

private[cpd] class NonRigidCPD[D: NDSpace](
    override val targetPoints: Seq[Point[D]],
    override val cpd: CPDFactory[D]
)(
    implicit vectorizer: Vectorizer[Point[D]],
    dataConverter: PointSequenceConverter[D]
) extends RigidCPD[D](targetPoints, cpd) {
  import cpd._
  override def Maximization(X: DenseMatrix[Double], Y: DenseMatrix[Double], P: DenseMatrix[Double], sigma2: Double): (DenseMatrix[Double], Double) = {
    // Update transform
    val P1: DenseVector[Double] = sum(P, Axis._1)
    val Pt1 = sum(P, Axis._0)
    val Np = sum(P1)

    val A: DenseMatrix[Double] = diag(P1) * G + lamdba * sigma2 * DenseMatrix.eye[Double](M)
    val B: DenseMatrix[Double] = P * X - diag(P1) * Y

    val W = A \ B
    // Update Point Cloud
    val TY = Y + G * W

    // Update variance
    /*
        Update the variance of the mixture model using the new estimate of the deformable transformation.
        See the update rule for sigma2 in Eq. 23 of of https://arxiv.org/pdf/0905.2635.pdf.
     */
    // The original CPD paper does not explicitly calculate the objective functional.
    // This functional will include terms from both the negative log-likelihood and
    // the Gaussian kernel used for regularization.
    val xPx: Double = Pt1.t dot sum(X *:* X, Axis._1)
    val yPy: Double = P1.t * sum(TY *:* TY, Axis._1)

    val trPXY: Double = sum(TY *:* (P * X))

    val updatedSigma2 = (xPx - 2 * trPXY + yPy) / (Np * dim)

    (TY, updatedSigma2)
  }
}
