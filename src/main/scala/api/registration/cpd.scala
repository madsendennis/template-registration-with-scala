package api.registration

import breeze.linalg.{*, Axis, DenseMatrix, DenseVector, InjectNumericOps, det, diag, inv, norm, sum, svd, tile, trace}
import breeze.numerics.{abs, pow}
import scalismo.common._
import scalismo.geometry._
import scalismo.mesh.{TriangleMesh, TriangleMesh3D}

/*
 Implementation of Point Set Registration: Coherent Point Drift
 In this script, only the non-rigid algorithm is implemented. Paper: https://arxiv.org/pdf/0905.2635.pdf
 A python implementation already exists from where parts of the implementation is from: https://github.com/siavashk/pycpd
 */


trait EM[D, DDomain[D] <: DiscreteDomain[D]] {
  def Registration(max_iteration: Int, tolerance: Double): DDomain[D]

  def Iteration(X: DenseMatrix[Double], Y: DenseMatrix[Double], sigma2: Double): (DenseMatrix[Double], Double)

  def Expectation(X: DenseMatrix[Double], Y: DenseMatrix[Double], sigma2: Double): DenseMatrix[Double]

  def Maximization(X: DenseMatrix[Double], Y: DenseMatrix[Double], P: DenseMatrix[Double], sigma2: Double): (DenseMatrix[Double], Double)
}

trait DiscreteDomainConverter[D, DDomain[D] <: DiscreteDomain[D]] {
  def denseMatrixToDomain(mat: DenseMatrix[Double], reference: DDomain[D]): DDomain[D]

  def PointCloudToMatrix(dc: DDomain[D]): DenseMatrix[Double] = {
    val dim: Int = dc.pointSet.points.toIndexedSeq.head.dimensionality
    val mat = DenseMatrix.zeros[Double](dc.pointSet.numberOfPoints, dim)
    dc.pointSet.points.zipWithIndex.foreach { case (p, i) =>
      (0 until dim).foreach { j =>
        mat(i, j) = p(j)
      }
    }
    mat
  }
}

object DiscreteDomainConverter {

  private def matrixTo1Dpoints(mat: DenseMatrix[Double]): IndexedSeq[Point[_1D]] = {
    0 until mat.rows map { r => Point1D(x = mat(r, 0)) }
  }

  private def matrixTo2Dpoints(mat: DenseMatrix[Double]): IndexedSeq[Point[_2D]] = {
    0 until mat.rows map { r => Point2D(x = mat(r, 0), y = mat(r, 1)) }
  }

  private def matrixTo3Dpoints(mat: DenseMatrix[Double]): IndexedSeq[Point[_3D]] = {
    0 until mat.rows map { r => Point3D(x = mat(r, 0), y = mat(r, 1), z = mat(r, 2)) }
  }

  implicit object denseMatrixToPointDomain1D extends DiscreteDomainConverter[_1D, UnstructuredPointsDomain] {
    override def denseMatrixToDomain(mat: DenseMatrix[Double], reference: UnstructuredPointsDomain[_1D]): UnstructuredPointsDomain[_1D] = {
      val p: IndexedSeq[Point[_1D]] = matrixTo1Dpoints(mat)
      UnstructuredPointsDomain.Create.CreateUnstructuredPointsDomain1D.create(p)
    }
  }

  implicit object denseMatrixToPointDomain2D extends DiscreteDomainConverter[_2D, UnstructuredPointsDomain] {
    override def denseMatrixToDomain(mat: DenseMatrix[Double], reference: UnstructuredPointsDomain[_2D]): UnstructuredPointsDomain[_2D] = {
      val p: IndexedSeq[Point[_2D]] = matrixTo2Dpoints(mat)
      UnstructuredPointsDomain.Create.CreateUnstructuredPointsDomain2D.create(p)
    }
  }

  implicit object denseMatrixToPointDomain3D extends DiscreteDomainConverter[_3D, UnstructuredPointsDomain] {
    override def denseMatrixToDomain(mat: DenseMatrix[Double], reference: UnstructuredPointsDomain[_3D]): UnstructuredPointsDomain[_3D] = {
      val p: IndexedSeq[Point[_3D]] = matrixTo3Dpoints(mat)
      UnstructuredPointsDomain.Create.CreateUnstructuredPointsDomain3D.create(p)
    }
  }

  implicit object denseMatrixToTriangleMesh3D extends DiscreteDomainConverter[_3D, TriangleMesh] {
    override def denseMatrixToDomain(mat: DenseMatrix[Double], reference: TriangleMesh[_3D]): TriangleMesh[_3D] = {
      val p: IndexedSeq[Point[_3D]] = matrixTo3Dpoints(mat)
      TriangleMesh3D(p, reference.triangulation)
    }
  }

  // TODO: Add remaining data types
}


class CPDNonRigid[D, DDomain[D] <: DiscreteDomain[D]](targetPoints: DDomain[D], referencePoints: DDomain[D], lamdba: Double = 2, beta: Double = 2, w: Double = 0)(implicit dataConverter: DiscreteDomainConverter[D, DDomain]) extends EM[D, DDomain] {
  // Paper: sigma2=3, lambda=1, beta=1
  // w: 0 until 1 - uniform distribution to account for outliers
  val M: Int = referencePoints.pointSet.numberOfPoints // num of reference points
  val N: Int = targetPoints.pointSet.numberOfPoints // num of target points
  val dim: Int = targetPoints.pointSet.points.toIndexedSeq.head.dimensionality // dimension

  val G: DenseMatrix[Double] = initializeKernelMatrixG(referencePoints, beta)

  val reference: DenseMatrix[Double] = dataConverter.PointCloudToMatrix(referencePoints)
  val target: DenseMatrix[Double] = dataConverter.PointCloudToMatrix(targetPoints)

  /*
  Initialize G matrix - formula in paper fig. 4
  */
  private def initializeKernelMatrixG(Y: DiscreteDomain[D], beta: Double): DenseMatrix[Double] = {
    val M = Y.pointSet.numberOfPoints
    val G: DenseMatrix[Double] = DenseMatrix.zeros[Double](M, M)
    (0 until M).map { i =>
      (0 until M).map { j =>
        G(i, j) = math.exp(-1 / (2 * math.pow(beta, 2)) * (Y.pointSet.point(PointId(i)) - Y.pointSet.point(PointId(j))).norm2)
      }
    }
    G
  }

  /*
  Initialize sigma2 - formula in paper fig. 4
   */
  private def initializeGaussianKernel(Y: DiscreteDomain[D], X: DiscreteDomain[D]): Double = {
    val M = Y.pointSet.numberOfPoints
    val N = X.pointSet.numberOfPoints
    val dim = X.pointSet.points.toIndexedSeq.head.dimensionality
    val s: Double = (0 until M).flatMap { m =>
      (0 until N).map { n =>
        (Y.pointSet.point(PointId(m)) - X.pointSet.point(PointId(n))).norm2
      }
    }.sum
    s / (dim * N * M)
  }

  override def Iteration(X: DenseMatrix[Double], Y: DenseMatrix[Double], sigma2: Double): (DenseMatrix[Double], Double) = {
    val P = Expectation(X, Y, sigma2)
    Maximization(X, Y, P, sigma2)
  }

  override def Registration(max_iteration: Int, tolerance: Double = 0.001): DDomain[D] = {
    val sigmaInit = initializeGaussianKernel(referencePoints, targetPoints)

    val fit = (0 until max_iteration).foldLeft((reference, sigmaInit)) { (it, i) =>
      val currentSigma2 = it._2
      println(s"CPD, iteration: ${i}, variance: ${currentSigma2}")
      val iter = Iteration(target, it._1, it._2)
      val TY = iter._1
      val newSigma2 = iter._2
      val diff = abs(newSigma2 - currentSigma2)
      if (diff < tolerance) {
        println("Converged")
        return dataConverter.denseMatrixToDomain(TY, referencePoints)
      }
      else {
        iter
      }
    }
    dataConverter.denseMatrixToDomain(fit._1, referencePoints)
  }

  override def Expectation(X: DenseMatrix[Double], Y: DenseMatrix[Double], sigma2: Double): DenseMatrix[Double] = {
    // TODO: Do matrix substraction with broadcasting instead
    val P: DenseMatrix[Double] = Y(*, ::).map { m =>
      val vec = X(*, ::).map { n =>
        val y = m.copy
        val x = n.copy
        math.exp(-1 / (2 * sigma2) * pow(norm(x - y), 2))
      }
      vec
    }

    val c = w / (1 - w) * math.pow((2 * math.Pi * sigma2), dim / 2) * (M / N)
    val denRow = DenseMatrix(sum(P, Axis._0).t)
    val den = tile(denRow, M, 1) + c

    P /:/ den
  }

  def Maximization(X: DenseMatrix[Double], Y: DenseMatrix[Double], P: DenseMatrix[Double], sigma2: Double): (DenseMatrix[Double], Double) = {
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


class CPDAffine[D, DDomain[D] <: DiscreteDomain[D]](targetPoints: DDomain[D], referencePoints: DDomain[D], lamdba: Double = 2, beta: Double = 2, w: Double = 0)(implicit dataConverter: DiscreteDomainConverter[D, DDomain]) extends CPDNonRigid[D, DDomain](targetPoints, referencePoints, lamdba, beta, w) {
  override def Maximization(X: DenseMatrix[Double], Y: DenseMatrix[Double], P: DenseMatrix[Double], sigma2: Double): (DenseMatrix[Double], Double) = {
    // Update transform
    val P1: DenseVector[Double] = sum(P, Axis._1)
    val Pt1 = sum(P, Axis._0)
    val Np = sum(P1)

    val muX = 1.0 / Np * X.t * P.t * DenseVector.ones[Double](P.rows)
    val muY = 1.0 / Np * Y.t * P1

    val Xhat = X - DenseVector.ones[Double](N) * muX.t
    val Yhat = Y - DenseVector.ones[Double](M) * muY.t

    val B = Xhat.t * P.t * Yhat * inv(Yhat.t * diag(P1) * Yhat)
    val t = muX - B * muY


    val s1 = trace(Xhat.t * diag(Pt1) * Xhat)
    val s2 = trace(Xhat.t * P.t * Yhat * B.t)
    val updatedSigma2 = 1 / (Np * dim) * (s1 - s2)
    val TY = Y * B.t + DenseVector.ones[Double](M) * t.t

    (TY, updatedSigma2)
  }
}


class CPDRigid[D, DDomain[D] <: DiscreteDomain[D]](targetPoints: DDomain[D], referencePoints: DDomain[D], lamdba: Double = 2, beta: Double = 2, w: Double = 0)(implicit dataConverter: DiscreteDomainConverter[D, DDomain]) extends CPDNonRigid[D, DDomain](targetPoints, referencePoints, lamdba, beta, w) {
  override def Maximization(X: DenseMatrix[Double], Y: DenseMatrix[Double], P: DenseMatrix[Double], sigma2: Double): (DenseMatrix[Double], Double) = {
    // Update transform
    val P1: DenseVector[Double] = sum(P, Axis._1)
    val Pt1 = sum(P, Axis._0)
    val Np = sum(P1)

    val muX = 1.0 / Np * X.t * P.t * DenseVector.ones[Double](P.rows)
    val muY = 1.0 / Np * Y.t * P1

    val Xhat = X - DenseVector.ones[Double](N) * muX.t
    val Yhat = Y - DenseVector.ones[Double](M) * muY.t

    val A = Xhat.t * P.t * Yhat
    val svd.SVD(u, _, v) = svd(A)
    val C = DenseVector.ones[Double](dim)
    C(dim - 1) = det(u * v.t)

    val R = u * diag(C) * v
    val s = trace(A.t * R) / trace(Yhat.t * diag(P1) * Yhat)
    val s1 = trace(Xhat.t * diag(Pt1) * Xhat)
    val s2 = s * trace(A.t * R)
    val t = muX - s * R * muY
    val updatedSigma2 = 1 / (Np * dim) * (s1 - s2)
    val TY = s * Y * R.t + DenseVector.ones[Double](M) * t.t

    (TY, updatedSigma2)
  }
}