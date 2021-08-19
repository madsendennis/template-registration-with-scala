package api.registration.config

import api.registration.utils.PointSequenceConverter
import api.{CorrespondencePairs, DefaultRegistrationPars, GiNGRConfig}
import breeze.linalg.{Axis, DenseMatrix, DenseVector, sum, tile}
import scalismo.common.PointId
import scalismo.geometry.Point.Point3DVectorizer
import scalismo.geometry.{Point, _3D}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.MultivariateNormalDistribution
import scalismo.utils.Memoize

case class expectationPars(template: Seq[Point[_3D]], target: Seq[Point[_3D]], sigma2: Double)


class CPD extends GiNGRConfig {
  val vectorizer: Point.Point3DVectorizer.type = Point3DVectorizer
  val dataConverter: PointSequenceConverter[_3D] = PointSequenceConverter.denseMatrixToPoint3DSequence
  var sigma2 = 100.0
  val w = 0.0
  val lambda = 1.0
  var P1inv: DenseVector[Double] = DenseVector.zeros[Double](0)
  var tarPoints: Seq[Point[_3D]] = Seq()
  var refPoints: Seq[Point[_3D]] = Seq()

  private val cashedP: Memoize[expectationPars, DenseMatrix[Double]] = Memoize(Expectation, 10)

  private def computeInitialSigma2(reference: Seq[Point[_3D]], target: Seq[Point[_3D]]): Double = {
    val N = target.length
    val M = reference.length
    val sumDist = reference.flatMap { pm =>
      target.map { pn =>
        (pn - pm).norm2
      }
    }.sum
    sumDist / (3.0 * N * M)
  }

  private def gaussKernel(x: Point[_3D], y: Point[_3D], sigma2: Double): Double = {
    math.exp(-(x - y).norm2 / (2.0 * sigma2))
  }

  private def Expectation(pars: expectationPars): DenseMatrix[Double] = {
    val reference = pars.template
    val target = pars.target
    val sigma2 = pars.sigma2
    val M = reference.length
    val N = target.length
    // TODO: Approximate using nyström
    val P: DenseMatrix[Double] = DenseMatrix.zeros[Double](M, N)
    reference.zipWithIndex.par.foreach { case (y, i) =>
      target.zipWithIndex.foreach { case (x, j) =>
        P(i, j) = gaussKernel(x, y, sigma2)
      }
    }
    val c = w / (1 - w) * math.pow((2.0 * math.Pi * sigma2), 3.0 / 2.0) * (M.toDouble / N.toDouble)
    val denRow = DenseMatrix(sum(P, Axis._0).t)
    val den = tile(denRow, M, 1) + c

    P /:/ den
  }

  private def computeSigma2(X: DenseMatrix[Double], TY: DenseMatrix[Double], P: DenseMatrix[Double]): Double = {
    val P1 = sum(P, Axis._1)
    val Pt1 = sum(P, Axis._0)
    val Np = sum(P1)

    val xPx: Double = Pt1.t dot sum(TY *:* TY, Axis._1)
    val yPy: Double = P1.t * sum(X *:* X, Axis._1)
    val trPXY: Double = sum(X *:* (P * TY))
    (xPx - 2 * trPXY + yPy) / (Np * 3.0)
  }

  override def Initialize(reference: TriangleMesh[_3D], target: TriangleMesh[_3D], default: DefaultRegistrationPars): Unit = {
    sigma2 = computeInitialSigma2(reference.pointSet.points.toSeq, target.pointSet.points.toSeq)
  }

  override def GetCorrespondence(reference: TriangleMesh[_3D], target: TriangleMesh[_3D]): CorrespondencePairs = {
    refPoints = reference.pointSet.points.toSeq
    tarPoints = target.pointSet.points.toSeq
    val P = cashedP(expectationPars(refPoints, tarPoints, sigma2))
    P1inv = 1.0 / sum(P, Axis._1)

    val deform = refPoints.zipWithIndex.par.map { case (y, i) =>
      val xscale = tarPoints.zipWithIndex.map { case (x, j) =>
        P1inv(i) * P(i, j) * x.toBreezeVector
      }
      vectorizer.unvectorize(sum(xscale) - y.toBreezeVector).toVector
    }
    val td = refPoints.zip(deform).map { case (p, d) => p + d }
    val corr = reference.pointSet.pointIds.toSeq.zip(td).map(t => (t._1, t._2)).toIndexedSeq
    CorrespondencePairs(corr)
  }

  override def UpdateUncertainty(meanUpdate: TriangleMesh[_3D]): Unit = {
    val P = cashedP(expectationPars(refPoints, tarPoints, sigma2))
    val TY = dataConverter.toMatrix(meanUpdate.pointSet.points.toSeq)
    val X = dataConverter.toMatrix(tarPoints)
    sigma2 = computeSigma2(TY, X, P)
    println(s"New uncertainty: ${sigma2}")
  }

  override def PointIdUncertainty(id: PointId): MultivariateNormalDistribution = {
    def d2MVND(d: Double): MultivariateNormalDistribution = {
      MultivariateNormalDistribution(DenseVector.zeros[Double](3), DenseMatrix.eye[Double](3) * d)
    }

    d2MVND(lambda * sigma2 * P1inv(id.id))
  }
}