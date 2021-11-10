package api.registration.config

import api.registration.utils.NonRigidClosestPointRegistrator.ClosestPointTriangleMesh3D
import api._
import breeze.linalg.{DenseMatrix, DenseVector}
import scalismo.common.PointId
import scalismo.statisticalmodel.MultivariateNormalDistribution

object ICPCorrespondence {
  def estimate[T](state: IcpRegistrationState): CorrespondencePairs = {
    val source = state.general.fit
    val target = state.general.target
    val corr = ClosestPointTriangleMesh3D.closestPointCorrespondence(source, target)
    CorrespondencePairs(pairs = corr._1.filter(_._3 == 1.0).map(f => (f._1, f._2)).toIndexedSeq)
  }
}

case class IcpConfiguration(
  override val maxIterations: Int = 100,
  override val threshold: Double = 1e-5,
  override val converged: (GeneralRegistrationState, GeneralRegistrationState) => Boolean = (last: GeneralRegistrationState, current: GeneralRegistrationState) => false,
  override val useLandmarkCorrespondence: Boolean = true,
  initialSigma: Double = 100.0,
  endSigma: Double = 1.0
) extends GingrConfig {
  val sigmaStep: Double = (initialSigma - endSigma) / maxIterations.toDouble
}

case class IcpRegistrationState(general: GeneralRegistrationState, config: IcpConfiguration) extends GingrRegistrationState[IcpRegistrationState] {
  override def updateGeneral(update: GeneralRegistrationState): IcpRegistrationState = this.copy(general = update)
}

object IcpRegistrationState {
  def apply(general: GeneralRegistrationState, config: IcpConfiguration): IcpRegistrationState = {
    val newGeneral = general.copy(
//      maxIterations = config.maxIterations,
      sigma2 = config.initialSigma
//      converged = false
    )
    new IcpRegistrationState(
      newGeneral,
      config
    )
  }
}

class IcpRegistration(
  override val getCorrespondence: IcpRegistrationState => CorrespondencePairs = (state: IcpRegistrationState) => ICPCorrespondence.estimate(state),
  override val getUncertainty: (PointId, IcpRegistrationState) => MultivariateNormalDistribution = (id: PointId, state: IcpRegistrationState) =>
    MultivariateNormalDistribution(DenseVector.zeros[Double](3), DenseMatrix.eye[Double](3) * state.general.sigma2)
) extends GingrAlgorithm[IcpRegistrationState] {
  def name = "ICP"
  // possibility to override the update function, or just use the base class method?
  override def updateSigma2(current: IcpRegistrationState): IcpRegistrationState = {
    val newGeneral = current.general.copy(sigma2 = current.general.sigma2 - current.config.sigmaStep)
    current.copy(general = newGeneral)
  }
}
