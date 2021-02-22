package api.registration.cpd

import api.registration.utils.PointSequenceConverter
import breeze.linalg.{Axis, DenseMatrix, DenseVector, diag, inv, sum}
import scalismo.common.Vectorizer
import scalismo.geometry.{NDSpace, Point}


/*
 Implementation of Bayesian Coherent Point Drift (BCPD)
 Paper: https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=8985307
 */
class BCPD[D: NDSpace](
                              val templatePoints: Seq[Point[D]],
                              val lambda: Double = 2,
                              val beta: Double = 2,
                              val w: Double = 0
                            )(
                              implicit val vectorizer: Vectorizer[Point[D]],
                              dataConverter: PointSequenceConverter[D]
                            ) {
  val M: Int = templatePoints.length // num of reference points
  val dim: Int = vectorizer.dim // dimension
  val G: DenseMatrix[Double] = initializeKernelMatrixG(templatePoints, beta)
  val template: DenseMatrix[Double] = dataConverter.toMatrix(templatePoints)

  require(0.0<=w && w<=1.0)
  require(beta>0)
  require(lambda>0)
  /**
    * Initialize G matrix - formula in paper fig. 4
    *
    * @param points
    * @param beta
    * @return
    */
  private def initializeKernelMatrixG(
                                       points: Seq[Point[D]],
                                       beta: Double
                                     ): DenseMatrix[Double] = {
    val M = points.length
    val G: DenseMatrix[Double] = DenseMatrix.zeros[Double](M, M)
    (0 until M).map { i =>
      (0 until M).map { j =>
        G(i, j) = math.exp(-(points(i) - points(j)).norm2 / (2*math.pow(beta, 2)))
      }
    }
    G
  }


  def computeSigma2(template: Seq[Point[D]], target: Seq[Point[D]]): Double = {
    val sumDist = template.toIndexedSeq.flatMap { pm =>
      target.toIndexedSeq.map { pn =>
        (pn - pm).norm2
      }
    }.sum
    sumDist / (3 * template.length * target.length)
  }

  override def Maximization(X: DenseMatrix[Double], Y: DenseMatrix[Double], P: DenseMatrix[Double], sigma2: Double): (DenseMatrix[Double], Double) = {
    // Update transform
    val P1: DenseVector[Double] = sum(P, Axis._1)
    val Pt1 = sum(P, Axis._0)
    val Np = sum(P1)

    val myG = G
    val diagP1inv = inv(diag(P1))
    val PX = P*X

    val A: DenseMatrix[Double] = G+lambda*sigma2*diagP1inv
    val B: DenseMatrix[Double] = diagP1inv*PX-Y

    val W = A \ B
    // Update Point Cloud
    val deform = myG * W
    val TY = Y + deform

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

//    val Xtarget: Seq[Point[D]] = dataConverter.toPointSequence(X)(vectorizer)
//    val Ytemp   = dataConverter.toPointSequence(TY)(vectorizer)
//
//    val updatedSigma2 = computeSigma2(Xtarget, Ytemp)

    (TY, updatedSigma2)
  }
}
