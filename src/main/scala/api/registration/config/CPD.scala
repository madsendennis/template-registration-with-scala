package api.registration.config

import api.registration.utils.PointSequenceConverter
import api.{CorrespondencePairs, GingrAlgorithm, GingrConfig, GingrRegistrationState}
import breeze.linalg.{Axis, DenseMatrix, DenseVector, sum, tile}
import scalismo.common.PointId
import scalismo.geometry.Point.Point3DVectorizer
import scalismo.geometry.{Point, _3D}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.{MultivariateNormalDistribution, PointDistributionModel}
import scalismo.transformations.{RigidTransformation, TranslationAfterRotationSpace3D}

object CPDCorrespondence {
  val vectorizer: Point.Point3DVectorizer.type = Point3DVectorizer

  def estimate[T](state: CpdRegistrationState): CorrespondencePairs = {
    val refPoints = state.fit.pointSet.points.toSeq
    val tarPoints = state.target.pointSet.points.toSeq
    val P = state.P
    val P1inv = 1.0 / sum(P, Axis._1)

    val deform = refPoints.zipWithIndex.par.map { case (y, i) =>
      val xscale = tarPoints.zipWithIndex.map { case (x, j) =>
        P1inv(i) * P(i, j) * x.toBreezeVector
      }
      vectorizer.unvectorize(sum(xscale) - y.toBreezeVector).toVector
    }
    val td = refPoints.zip(deform).map { case (p, d) => p + d }
    val corr = state.fit.pointSet.pointIds.toSeq.zip(td).map(t => (t._1, t._2)).toIndexedSeq
    CorrespondencePairs(corr)
  }
}

case class CpdRegistrationState(
                                 override val model: PointDistributionModel[_3D, TriangleMesh],
                                 override val modelParameters: DenseVector[Double],
                                 override val target: TriangleMesh[_3D],
                                 override val fit: TriangleMesh[_3D],
                                 override val rigidAlignment: RigidTransformation[_3D],
                                 override val scaling: Double = 1.0,
                                 override val converged: Boolean,
                                 sigma2: Double,
                                 w: Double = 0.0,
                                 lambda: Double = 1.0,
                                 P: DenseMatrix[Double],
                                 override val iteration: Int = 0
                               ) extends GingrRegistrationState[CpdRegistrationState] {
  override def updateFit(next: TriangleMesh[_3D]): CpdRegistrationState = this.copy(fit = next)
}

case class CpdConfiguration(
                             override val maxIterations: Int = 100,
                             override val converged: (CpdRegistrationState, CpdRegistrationState) => Boolean = (last: CpdRegistrationState, current: CpdRegistrationState) => {
                               last.sigma2 == current.sigma2
                             },
                             initialSigma: Option[Double] = None,
                             w: Double = 0.0,
                             lambda: Double = 1.0
                           ) extends GingrConfig[CpdRegistrationState] {}

class CpdRegistration(
                       val target: TriangleMesh[_3D],
                       val config: CpdConfiguration,
                       val pdm: PointDistributionModel[_3D, TriangleMesh],
                       override val getCorrespondence: CpdRegistrationState => CorrespondencePairs = (state: CpdRegistrationState) => CPDCorrespondence.estimate(state),
                       override val getUncertainty: (PointId, CpdRegistrationState) => MultivariateNormalDistribution = (id: PointId, state: CpdRegistrationState) => {
                         val P1inv = 1.0 / sum(state.P, Axis._1)
                         MultivariateNormalDistribution(DenseVector.zeros[Double](3), DenseMatrix.eye[Double](3) * state.sigma2 * state.lambda * P1inv(id.id))
                       }
                     ) extends GingrAlgorithm[CpdRegistrationState] {

  val dataConverter: PointSequenceConverter[_3D] = PointSequenceConverter.denseMatrixToPoint3DSequence

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

  override def initialize(): CpdRegistrationState = {
    val initSigma = config.initialSigma.getOrElse(computeInitialSigma2(pdm.mean.pointSet.points.toSeq, target.pointSet.points.toSeq))
    val initial =
      CpdRegistrationState(
        pdm,
        DenseVector.zeros[Double](pdm.rank),
        target,
        pdm.mean,
        TranslationAfterRotationSpace3D(Point(0, 0, 0)).identityTransformation,
        1.0,
        false,
        initSigma,
        config.w,
        config.lambda,
        DenseMatrix.zeros[Double](0,0),
        config.maxIterations
      )
    initial
  }


  private def updateSigma(current: CpdRegistrationState, meanUpdate: Seq[Point[_3D]]): Double = {
    val P = current.P
    val X = dataConverter.toMatrix(current.target.pointSet.points.toSeq)
    val TY = dataConverter.toMatrix(meanUpdate)
    val P1 = sum(P, Axis._1)
    val Pt1 = sum(P, Axis._0)
    val Np = sum(P1)

    val xPx: Double = Pt1.t dot sum(X *:* X, Axis._1)
    val yPy: Double = P1.t * sum(TY *:* TY, Axis._1)
    val trPXY: Double = sum(TY *:* (P * X))
    (xPx - 2 * trPXY + yPy) / (Np * 3.0)
  }

  private def gaussKernel(x: Point[_3D], y: Point[_3D], sigma2: Double): Double = {
    math.exp(-(x - y).norm2 / (2.0 * sigma2))
  }

  private def Expectation(current: CpdRegistrationState): DenseMatrix[Double] = {
    val refPoints = current.fit.pointSet.points.toSeq
    val tarPoints = current.target.pointSet.points.toSeq
    val M = refPoints.length
    val N = tarPoints.length
    // TODO: Approximate using nyström
    val P: DenseMatrix[Double] = DenseMatrix.zeros[Double](M, N)
    refPoints.zipWithIndex.par.foreach { case (y, i) =>
      tarPoints.zipWithIndex.foreach { case (x, j) =>
        P(i, j) = gaussKernel(x, y, current.sigma2)
      }
    }
    val c = current.w / (1 - current.w) * math.pow((2.0 * math.Pi * current.sigma2), 3.0 / 2.0) * (M.toDouble / N.toDouble)
    val denRow = DenseMatrix(sum(P, Axis._0).t)
    val den = tile(denRow, M, 1) + c

    P /:/ den
  }

  // possibility to override the update function, or just use the base class method?
  override def update(current: CpdRegistrationState): CpdRegistrationState = {
    println(s"iteration: ${current.iteration}, sigma: ${current.sigma2}")
    val currentP = current.copy(P = Expectation(current))
    val correspondences = getCorrespondence(currentP)
    val uncertainObservations = correspondences.pairs.map { pair =>
      val (pid, point) = pair
      val uncertainty = getUncertainty(pid, currentP)
      (pid, point, uncertainty)
    }
    val mean = current.model.posterior(uncertainObservations).mean
    val (model, alpha, fit) = if(true) {
      val alpha = current.model.coefficients(mean)
      (current.model, alpha, mean)
    }
    else{
      val model = updateModel(current.model, mean)
      (model, current.modelParameters, model.mean)
    }
    currentP.copy(
      model = model,
      modelParameters = alpha,
      fit = fit,
      sigma2 = updateSigma(currentP, fit.pointSet.points.toSeq),
      iteration = currentP.iteration - 1
    )
  }
}

