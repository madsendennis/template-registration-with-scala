package api.registration.config

import api.registration.utils.PointSequenceConverter
import api.{CorrespondencePairs, GingrAlgorithm, GingrConfig, GingrState, GlobalTranformationType, RigidTransforms}
import breeze.linalg.{sum, tile, Axis, DenseMatrix, DenseVector}
import scalismo.common.PointId
import scalismo.geometry.Point.Point3DVectorizer
import scalismo.geometry.{_3D, Landmark, Point}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.{MultivariateNormalDistribution, PointDistributionModel}
import scalismo.transformations.{TranslationAfterRotation, TranslationAfterRotationSpace3D}

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
  override val modelLandmarks: Option[Seq[Landmark[_3D]]] = None,
  override val target: TriangleMesh[_3D],
  override val targetLandmarks: Option[Seq[Landmark[_3D]]] = None,
  override val fit: TriangleMesh[_3D],
  override val alignment: TranslationAfterRotation[_3D],
  override val scaling: Double = 1.0,
  override val converged: Boolean,
  override val sigma2: Double,
  override val iteration: Int = 0,
  override val globalTransformation: GlobalTranformationType = RigidTransforms,
  config: CpdConfiguration
) extends GingrState[CpdRegistrationState] {
  override def updateFit(next: TriangleMesh[_3D]): CpdRegistrationState = this.copy(fit = next)
  override private[api] def updateAlignment(next: TranslationAfterRotation[_3D]): CpdRegistrationState = this.copy(alignment = next)
  override private[api] def updateScaling(next: Double): CpdRegistrationState = this.copy(scaling = next)
  override private[api] def updateModelParameters(next: DenseVector[Double]): CpdRegistrationState = this.copy(modelParameters = next)
  override private[api] def updateIteration(next: Int): CpdRegistrationState = this.copy(iteration = next)
  val P: DenseMatrix[Double] = {
    def gaussKernel(x: Point[_3D], y: Point[_3D], sigma2: Double): Double = {
      math.exp(-(x - y).norm2 / (2.0 * sigma2))
    }
    val refPoints = fit.pointSet.points.toSeq
    val tarPoints = target.pointSet.points.toSeq
    val M = refPoints.length
    val N = tarPoints.length
    // TODO: Approximate using nyström
    val P: DenseMatrix[Double] = DenseMatrix.zeros[Double](M, N)
    refPoints.zipWithIndex.par.foreach { case (y, i) =>
      tarPoints.zipWithIndex.foreach { case (x, j) =>
        P(i, j) = gaussKernel(x, y, sigma2)
      }
    }
    val c = config.w / (1 - config.w) * math.pow((2.0 * math.Pi * sigma2), 3.0 / 2.0) * (M.toDouble / N.toDouble)
    val denRow = DenseMatrix(sum(P, Axis._0).t)
    val den = tile(denRow, M, 1) + c

    P /:/ den
  }
}

object CpdRegistrationState {

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

  def apply(model: PointDistributionModel[_3D, TriangleMesh], target: TriangleMesh[_3D], config: CpdConfiguration): CpdRegistrationState = {
    apply(model, None, target, None, config)
  }

  def apply(
    model: PointDistributionModel[_3D, TriangleMesh],
    modelLandmarks: Option[Seq[Landmark[_3D]]],
    target: TriangleMesh[_3D],
    targetLandmarks: Option[Seq[Landmark[_3D]]],
    config: CpdConfiguration): CpdRegistrationState = {
    val initial =
      new CpdRegistrationState(
        model = model,
        modelParameters = DenseVector.zeros[Double](model.rank),
        modelLandmarks = modelLandmarks,
        target = target,
        targetLandmarks = targetLandmarks,
        fit = model.mean,
        alignment = TranslationAfterRotationSpace3D(Point(0, 0, 0)).identityTransformation,
        scaling = 1.0,
        converged = false,
        sigma2 = config.initialSigma.getOrElse(computeInitialSigma2(model.mean.pointSet.points.toSeq, target.pointSet.points.toSeq)),
        iteration = config.maxIterations,
        config = config
      )
    initial
  }

  def apply[T](state: GingrState[T], config: CpdConfiguration): CpdRegistrationState = {
    new CpdRegistrationState(
      state.model(),
      state.modelParameters(),
      state.modelLandmarks(),
      state.target(),
      state.targetLandmarks(),
      state.fit(),
      state.alignment(),
      state.scaling(),
      state.converged(),
      state.sigma2(),
      config.maxIterations,
      state.globalTransformation(),
      config
    )
  }
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
  override val getCorrespondence: CpdRegistrationState => CorrespondencePairs = (state: CpdRegistrationState) => CPDCorrespondence.estimate(state),
  override val getUncertainty: (PointId, CpdRegistrationState) => MultivariateNormalDistribution = (id: PointId, state: CpdRegistrationState) => {
    val P = state.P
    val P1inv = 1.0 / sum(P, Axis._1)
    MultivariateNormalDistribution(DenseVector.zeros[Double](3), DenseMatrix.eye[Double](3) * state.sigma2 * state.config.lambda * P1inv(id.id))
  }
) extends GingrAlgorithm[CpdRegistrationState] {
  val dataConverter: PointSequenceConverter[_3D] = PointSequenceConverter.denseMatrixToPoint3DSequence

  // possibility to override the update function, or just use the base class method?
  override def updateSigma2(current: CpdRegistrationState): CpdRegistrationState = {
    val meanUpdate = current.fit.pointSet.points.toSeq
    val P = current.P
    val X = dataConverter.toMatrix(current.target.pointSet.points.toSeq)
    val TY = dataConverter.toMatrix(meanUpdate)
    val P1 = sum(P, Axis._1)
    val Pt1 = sum(P, Axis._0)
    val Np = sum(P1)

    val xPx: Double = Pt1.t.dot(sum(X *:* X, Axis._1))
    val yPy: Double = P1.t * sum(TY *:* TY, Axis._1)
    val trPXY: Double = sum(TY *:* (P * X))
    val sigma2 = (xPx - 2 * trPXY + yPy) / (Np * 3.0)
    current.copy(sigma2 = sigma2)
  }
}
