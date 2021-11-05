package api.sampling.generators

import api.{EulerRotation, GingrRegistrationState}
import scalismo.sampling.{ProposalGenerator, TransitionProbability}

sealed trait RotationAxis

case object RollAxis extends RotationAxis

case object PitchAxis extends RotationAxis

case object YawAxis extends RotationAxis

case class GaussianAxisRotationProposal[State <: GingrRegistrationState[State]](sdevRot: Double, axis: RotationAxis, generatedBy: String = "RotationProposal")
  extends ProposalGenerator[State] with TransitionProbability[State] {

  val perturbationDistr = new breeze.stats.distributions.Gaussian(0, sdevRot)

  override def propose(theta: State): State = {
    val rotParams = theta.general.modelParameters.pose.rotation.angles
    val newRotParams = axis match {
      case RollAxis => rotParams.copy(phi = rotParams.phi + perturbationDistr.sample())
      case PitchAxis => rotParams.copy(theta = rotParams.theta + perturbationDistr.sample())
      case YawAxis => rotParams.copy(psi = rotParams.psi + perturbationDistr.sample())
    }
    val newRotation = EulerRotation(newRotParams, theta.general.modelParameters.pose.rotation.center)
    theta.updateGeneral(theta.general.updateRotation(newRotation).updateGeneratedBy(generatedBy))
  }

  override def logTransitionProbability(from: State, to: State): Double = {
//    if (to.copy(poseParameters = to.poseParameters.copy(rotation = from.poseParameters.rotation)).allParameters != from.allParameters) {
//      Double.NegativeInfinity
//    } else {
    val rotParamsFrom = from.general.modelParameters.pose.rotation.angles
    val rotParamsTo = to.general.modelParameters.pose.rotation.angles
    val residual = axis match {
      case RollAxis => rotParamsTo.phi - rotParamsFrom.phi
      case PitchAxis => rotParamsTo.theta - rotParamsFrom.theta
      case YawAxis => rotParamsTo.psi - rotParamsFrom.psi
    }
    perturbationDistr.logPdf(residual)
//    }
  }
}

case class GaussianAxisTranslationProposal[State <: GingrRegistrationState[State]](sdevTrans: Double, axis: Int, generatedBy: String = "TranslationProposal")
  extends ProposalGenerator[State] with TransitionProbability[State] {
  require(axis < 3)
  val perturbationDistr = new breeze.stats.distributions.Gaussian(0, sdevTrans)

  override def propose(theta: State): State = {

    perturbationDistr.sample()

    val transParams = theta.general.modelParameters.pose.translation
    val newTransParams = axis match {
      case 0 => transParams.copy(x = transParams(axis) + perturbationDistr.sample())
      case 1 => transParams.copy(y = transParams(axis) + perturbationDistr.sample())
      case 2 => transParams.copy(z = transParams(axis) + perturbationDistr.sample())
    }
    theta.updateGeneral(theta.general.updateTranslation(newTransParams).updateGeneratedBy(generatedBy))
  }

  override def logTransitionProbability(from: State, to: State): Double = {
//    if (to.copy(poseParameters = to.poseParameters.copy(translation = from.poseParameters.translation)).allParameters != from.allParameters) {
//      Double.NegativeInfinity
//    } else {
    val residual = to.general.modelParameters.pose.translation(axis) - from.general.modelParameters.pose.translation(axis)
    perturbationDistr.logPdf(residual)
//    }
  }
}
