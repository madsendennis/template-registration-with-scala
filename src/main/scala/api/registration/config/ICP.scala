package api.registration.config

import api.registration.utils.NonRigidClosestPointRegistrator.ClosestPointTriangleMesh3D
import api.{CorrespondencePairs, GingrAlgorithm, GingrConfig, GingrState, GlobalTranformationType, NoTransforms}
import breeze.linalg.{DenseMatrix, DenseVector}
import scalismo.common.PointId
import scalismo.geometry.{_3D, Landmark, Point}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.{MultivariateNormalDistribution, PointDistributionModel}
import scalismo.transformations.{TranslationAfterRotation, TranslationAfterRotationSpace3D}

object ICPCorrespondence {
  def estimate[T](state: GingrState[T]): CorrespondencePairs = {
    val source = state.fit()
    val target = state.target()
    val corr = ClosestPointTriangleMesh3D.closestPointCorrespondence(source, target)
    CorrespondencePairs(pairs = corr._1.filter(_._3 == 1.0).map(f => (f._1, f._2)).toIndexedSeq)
  }
}

case class IcpConfiguration(
  override val maxIterations: Int = 100,
  override val converged: (IcpRegistrationState, IcpRegistrationState) => Boolean = (last: IcpRegistrationState, current: IcpRegistrationState) => false,
  initialSigma: Double = 100.0,
  endSigma: Double = 1.0,
  globalTransformation: GlobalTranformationType = NoTransforms
) extends GingrConfig[IcpRegistrationState] {
  val sigmaStep: Double = (initialSigma - endSigma) / maxIterations.toDouble
}

case class IcpRegistrationState(
  override val model: PointDistributionModel[_3D, TriangleMesh],
  override val modelParameters: DenseVector[Double],
  override val modelLandmarks: Option[Seq[Landmark[_3D]]] = None,
  override val target: TriangleMesh[_3D],
  override val targetLandmarks: Option[Seq[Landmark[_3D]]] = None,
  override val fit: TriangleMesh[_3D],
  override val alignment: TranslationAfterRotation[_3D],
  override val scaling: Double = 1.0,
  override val converged: Boolean,
  override val sigma2: Double = 1.0,
  override val iteration: Int = 0,
  override val globalTransformation: GlobalTranformationType = NoTransforms,
  config: IcpConfiguration
) extends GingrState[IcpRegistrationState] {
  override def updateFit(next: TriangleMesh[_3D]): IcpRegistrationState = this.copy(fit = next)
  override private[api] def updateAlignment(next: TranslationAfterRotation[_3D]): IcpRegistrationState = this.copy(alignment = next)
  override private[api] def updateScaling(next: Double): IcpRegistrationState = this.copy(scaling = next)
  override private[api] def updateModelParameters(next: DenseVector[Double]): IcpRegistrationState = this.copy(modelParameters = next)
  override private[api] def updateIteration(next: Int): IcpRegistrationState = this.copy(iteration = next)
}

object IcpRegistrationState {
  def apply(model: PointDistributionModel[_3D, TriangleMesh], target: TriangleMesh[_3D], config: IcpConfiguration): IcpRegistrationState = {
    apply(model, None, target, None, config)
  }

  def apply(
    model: PointDistributionModel[_3D, TriangleMesh],
    modelLandmarks: Option[Seq[Landmark[_3D]]],
    target: TriangleMesh[_3D],
    targetLandmarks: Option[Seq[Landmark[_3D]]],
    config: IcpConfiguration): IcpRegistrationState = {
    val initial =
      new IcpRegistrationState(
        model = model,
        modelParameters = DenseVector.zeros[Double](model.rank),
        modelLandmarks = modelLandmarks,
        target = target,
        targetLandmarks = targetLandmarks,
        fit = model.mean,
        alignment = TranslationAfterRotationSpace3D(Point(0, 0, 0)).identityTransformation,
        scaling = 1.0,
        converged = false,
        sigma2 = config.initialSigma,
        iteration = config.maxIterations,
        config = config
      )
    initial
  }

  def apply[T](state: GingrState[T], config: IcpConfiguration): IcpRegistrationState = {
    new IcpRegistrationState(
      state.model(),
      state.modelParameters(),
      state.modelLandmarks(),
      state.target(),
      state.targetLandmarks(),
      state.fit(),
      state.alignment(),
      state.scaling(),
      state.converged(),
      config.initialSigma,
      config.maxIterations,
      state.globalTransformation(),
      config
    )
  }
}

class IcpRegistration(
  override val getCorrespondence: IcpRegistrationState => CorrespondencePairs = (state: IcpRegistrationState) => ICPCorrespondence.estimate(state),
  override val getUncertainty: (PointId, IcpRegistrationState) => MultivariateNormalDistribution = (id: PointId, state: IcpRegistrationState) =>
    MultivariateNormalDistribution(DenseVector.zeros[Double](3), DenseMatrix.eye[Double](3) * state.sigma2)
) extends GingrAlgorithm[IcpRegistrationState] {

  // possibility to override the update function, or just use the base class method?
  override def updateSigma2(current: IcpRegistrationState): IcpRegistrationState = {
    current.copy(sigma2 = current.sigma2 - current.config.sigmaStep)
  }
}
