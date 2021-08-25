package api.registration.config

import api.registration.utils.NonRigidClosestPointRegistrator.ClosestPointTriangleMesh3D
import api.{CorrespondencePairs, GingrAlgorithm, GingrConfig, GingrRegistrationState, GlobalTranformationType, NoTransforms, RigidTransforms}
import breeze.linalg.{DenseMatrix, DenseVector}
import scalismo.common.PointId
import scalismo.geometry.{Point, _3D}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.{MultivariateNormalDistribution, PointDistributionModel}
import scalismo.transformations.{RigidTransformation, TranslationAfterRotation, TranslationAfterRotationSpace3D}

object ICPCorrespondence {
  def estimate[T](state: GingrRegistrationState[T]): CorrespondencePairs = {
    val source = state.fit()
    val target = state.target()
    val corr = ClosestPointTriangleMesh3D.closestPointCorrespondence(source, target)
    CorrespondencePairs(pairs = corr._1.filter(_._3 == 1.0).map(f => (f._1, f._2)).toIndexedSeq)
  }
}

case class IcpRegistrationState(
                                 override val model: PointDistributionModel[_3D, TriangleMesh],
                                 override val modelParameters: DenseVector[Double],
                                 override val target: TriangleMesh[_3D],
                                 override val fit: TriangleMesh[_3D],
                                 override val alignment: TranslationAfterRotation[_3D],
                                 override val scaling: Double = 1.0,
                                 override val converged: Boolean,
                                 override val sigma2: Double = 1.0,
                                 override val iteration: Int = 0,
                                 override val globalTransformation: GlobalTranformationType = NoTransforms
                               ) extends GingrRegistrationState[IcpRegistrationState] {
  override def updateFit(next: TriangleMesh[_3D]): IcpRegistrationState = this.copy(fit = next)
  override private[api] def updateAlignment(next: TranslationAfterRotation[_3D]): IcpRegistrationState = this.copy(alignment = next)
  override private[api] def updateScaling(next: Double): IcpRegistrationState = this.copy(scaling = next)
  override private[api] def updateModelParameters(next: DenseVector[Double]): IcpRegistrationState = this.copy(modelParameters = next)
  override private[api] def updateIteration(next: Int): IcpRegistrationState = this.copy(iteration = next)
}

case class IcpConfiguration(
                             override val maxIterations: Int = 100,
                             override val converged: (IcpRegistrationState, IcpRegistrationState) => Boolean = (last: IcpRegistrationState, current: IcpRegistrationState) => false,
                             initialSigma: Double = 100.0,
                             endSigma: Double = 1.0
                           ) extends GingrConfig[IcpRegistrationState] {}

class IcpRegistration(
                       val target: TriangleMesh[_3D],
                       val config: IcpConfiguration,
                       val pdm: PointDistributionModel[_3D, TriangleMesh],
                       override val getCorrespondence: IcpRegistrationState => CorrespondencePairs = (state: IcpRegistrationState) => ICPCorrespondence.estimate(state),
                       override val getUncertainty: (PointId, IcpRegistrationState) => MultivariateNormalDistribution = (id: PointId, state: IcpRegistrationState) =>
                         MultivariateNormalDistribution(DenseVector.zeros[Double](3), DenseMatrix.eye[Double](3) * state.sigma2)
                     ) extends GingrAlgorithm[IcpRegistrationState] {

  override def initialize(): IcpRegistrationState = {
    val initial =
      IcpRegistrationState(
        pdm,
        DenseVector.zeros[Double](pdm.rank),
        target,
        pdm.mean,
        TranslationAfterRotationSpace3D(Point(0, 0, 0)).identityTransformation,
        1.0,
        false,
        config.initialSigma,
        config.maxIterations
      )
    initial
  }

  private val sigmaStep = (config.initialSigma - config.endSigma) / config.maxIterations.toDouble


  // possibility to override the update function, or just use the base class method?
  override def updateSigma2(current: IcpRegistrationState): IcpRegistrationState = {
    current.copy(sigma2 = current.sigma2 - sigmaStep)
  }
}

