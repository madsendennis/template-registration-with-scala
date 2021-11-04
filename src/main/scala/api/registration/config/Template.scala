package api.registration.config

import api.{CorrespondencePairs, GeneralRegistrationState, GingrAlgorithm, GingrConfig, GingrRegistrationState}
import breeze.linalg.{DenseMatrix, DenseVector}
import scalismo.common.PointId
import scalismo.statisticalmodel.MultivariateNormalDistribution

case class TemplateConfiguration(
  override val maxIterations: Int = 1,
  override val converged: (GeneralRegistrationState, GeneralRegistrationState) => Boolean = (last: GeneralRegistrationState, current: GeneralRegistrationState) => false,
  override val useLandmarkCorrespondence: Boolean = true
) extends GingrConfig {}

case class TemplateRegistrationState(general: GeneralRegistrationState, config: TemplateConfiguration) extends GingrRegistrationState[TemplateRegistrationState] {
  override private[api] def updateGeneral(update: GeneralRegistrationState): TemplateRegistrationState = this.copy(general = update)
}

object TemplateRegistrationState {
  def apply(general: GeneralRegistrationState, config: TemplateConfiguration): TemplateRegistrationState = {
    val newGeneral = general.copy(
      maxIterations = config.maxIterations,
      converged = false
    )
    new TemplateRegistrationState(
      newGeneral,
      config
    )
  }
}

class TemplateRegistration(
  override val getCorrespondence: TemplateRegistrationState => CorrespondencePairs = (state: TemplateRegistrationState) => CorrespondencePairs.empty()
) extends GingrAlgorithm[TemplateRegistrationState] {
  def name = "Template"
  override val getUncertainty: (PointId, TemplateRegistrationState) => MultivariateNormalDistribution = (id: PointId, state: TemplateRegistrationState) =>
    MultivariateNormalDistribution(DenseVector.zeros[Double](3), DenseMatrix.eye[Double](3))
}
