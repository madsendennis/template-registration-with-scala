package samplingtools.proposals

import api.sampling.ModelFittingParameters
import scalismo.sampling.proposals.MixtureProposal
import scalismo.sampling.proposals.MixtureProposal.implicits._
import scalismo.sampling.{ProposalGenerator, SymmetricTransitionRatio, TransitionProbability}
import scalismo.utils.Random

/**
  * Created by luetma00 on 21.10.16.
  */

sealed trait RotationAxis

case object RollAxis extends RotationAxis

case object PitchAxis extends RotationAxis

case object YawAxis extends RotationAxis


case class GaussianAxisRotationProposal(sdevRot: Double, axis: RotationAxis, generatedBy: String = "RotationProposal")
  extends ProposalGenerator[ModelFittingParameters] with TransitionProbability[ModelFittingParameters] {

  val perturbationDistr = new breeze.stats.distributions.Gaussian(0, sdevRot)


  override def propose(theta: ModelFittingParameters): ModelFittingParameters = {
    val rotParams = theta.poseParameters.rotation
    val newRotParams = axis match {
      case RollAxis => rotParams.copy(_1 = rotParams._1 + perturbationDistr.sample())
      case PitchAxis => rotParams.copy(_2 = rotParams._2 + perturbationDistr.sample())
      case YawAxis => rotParams.copy(_3 = rotParams._3 + perturbationDistr.sample())
    }

    theta.copy(poseParameters = theta.poseParameters.copy(rotation = newRotParams), generatedBy = generatedBy)
  }

  override def logTransitionProbability(from: ModelFittingParameters, to: ModelFittingParameters): Double = {
    val rotParamsFrom = from.poseParameters.rotation
    val rotParamsTo = to.poseParameters.rotation
    val residual = axis match {
      case RollAxis => rotParamsTo._1 - rotParamsFrom._1
      case PitchAxis => rotParamsTo._2 - rotParamsFrom._2
      case YawAxis => rotParamsTo._3 - rotParamsFrom._3
    }
    perturbationDistr.logPdf(residual)
  }
}


case class GaussianAxisTranslationProposal(sdevTrans: Double, axis: Int, generatedBy: String = "TranslationProposal")
  extends ProposalGenerator[ModelFittingParameters] with TransitionProbability[ModelFittingParameters] {

  require(axis < 3)
  val perturbationDistr = new breeze.stats.distributions.Gaussian(0, sdevTrans)


  override def propose(theta: ModelFittingParameters): ModelFittingParameters = {

    val transParams = theta.poseParameters.translation
    val newTransParams = axis match {
      case 0 => transParams.copy(x = transParams(axis) + perturbationDistr.sample())
      case 1 => transParams.copy(y = transParams(axis) + perturbationDistr.sample())
      case 2 => transParams.copy(z = transParams(axis) + perturbationDistr.sample())
    }
    theta.copy(poseParameters = theta.poseParameters.copy(translation = newTransParams), generatedBy = generatedBy)
  }

  override def logTransitionProbability(from: ModelFittingParameters, to: ModelFittingParameters) = {
    val residual = to.poseParameters.translation(axis) - from.poseParameters.translation(axis)
    perturbationDistr.logPdf(residual)
  }
}

case class StandardRandomPoseProposal(scaleFactor: Double) (implicit random: Random, generatedBy: String = "RandomPoseProposal")
  extends ProposalGenerator[ModelFittingParameters] with TransitionProbability[ModelFittingParameters] {

  val randomGaussianPoseProposal = {
    val yawProposalC = GaussianAxisRotationProposal(0.05 * scaleFactor, YawAxis, generatedBy = "RotationYaw-0.05")
    val yawProposalI = GaussianAxisRotationProposal(0.01 * scaleFactor, YawAxis, generatedBy = "RotationYaw-0.01")
    val yawProposalF = GaussianAxisRotationProposal(0.005 * scaleFactor, YawAxis, generatedBy = "RotationYaw-0.005")
    val rotationYaw = MixtureProposal(0.0 *: yawProposalC + 0.4 *: yawProposalI + 0.5 *: yawProposalF)

    val pitchProposalC = GaussianAxisRotationProposal(0.05 * scaleFactor, PitchAxis, generatedBy = "RotationPitch-0.05")
    val pitchProposalI = GaussianAxisRotationProposal(0.01 * scaleFactor, PitchAxis, generatedBy = "RotationPitch-0.01")
    val pitchProposalF = GaussianAxisRotationProposal(0.001 * scaleFactor, PitchAxis, generatedBy = "RotationPitch-0.001")
    val rotationPitch = MixtureProposal(0.0 *: pitchProposalC + 0.4 *: pitchProposalI + 0.5 *: pitchProposalF)

    val rollProposalC = GaussianAxisRotationProposal(0.2 * scaleFactor, RollAxis, generatedBy = "RotationRoll-0.2")
    val rollProposalI = GaussianAxisRotationProposal(0.1 * scaleFactor, RollAxis, generatedBy = "RotationRoll-0.1")
    val rollProposalF = GaussianAxisRotationProposal(0.01 * scaleFactor, RollAxis, generatedBy = "RotationRoll-0.01")
    val rotationRoll = MixtureProposal(0.1 *: rollProposalC + 0.4 *: rollProposalI + 0.5 *: rollProposalF)


    val rotationProposal = MixtureProposal(0.5 *: rotationRoll + 0.2 *: rotationPitch + 0.2 *: rotationYaw)


    val translationXProposalC = GaussianAxisTranslationProposal(10.0 * scaleFactor, 0, generatedBy = "TranslationX-10.0")
    val translationXProposalI = GaussianAxisTranslationProposal(3.0 * scaleFactor, 0, generatedBy = "TranslationX-3.0")
    val translationXProposalF = GaussianAxisTranslationProposal(1.0 * scaleFactor, 0, generatedBy = "TranslationX-1.0")
    val translationXProposal = MixtureProposal(0.0 *: translationXProposalC + 0.4 *: translationXProposalI + 0.5 *: translationXProposalF)


    val translationYProposalC = GaussianAxisTranslationProposal(10.0 * scaleFactor, 1, generatedBy = "TranslationY-10.0")
    val translationYProposalI = GaussianAxisTranslationProposal(3.0 * scaleFactor, 1, generatedBy = "TranslationY-3.0")
    val translationYProposalF = GaussianAxisTranslationProposal(1.0 * scaleFactor, 1, generatedBy = "TranslationY-1.0")
    val translationYProposal = MixtureProposal(0.0 *: translationYProposalC + 0.4 *: translationYProposalI + 0.5 *: translationYProposalF)


    val translationZProposalC = GaussianAxisTranslationProposal(10.0 * scaleFactor, 2, generatedBy = "TranslationZ-10.0")
    val translationZProposalI = GaussianAxisTranslationProposal(3.0 * scaleFactor, 2, generatedBy = "TranslationZ-3.0")
    val translationZProposalF = GaussianAxisTranslationProposal(1.0 * scaleFactor, 2, generatedBy = "TranslationZ-1.0")
    val translationZProposal = MixtureProposal(0.0 *: translationZProposalC + 0.4 *: translationZProposalI + 0.5 *: translationZProposalF)

    val translationProposal = MixtureProposal(0.3 *: translationXProposal + 0.3 *: translationYProposal + 0.4 *: translationZProposal)
    val poseProposal = MixtureProposal(0.4 *: rotationProposal + 0.4 *: translationProposal)

    poseProposal
  }

  override def propose(current: ModelFittingParameters): ModelFittingParameters = {
    val parameters = randomGaussianPoseProposal.propose(current)
    parameters//.copy(generatedBy = "StandardRandomPoseProposal")
  }

  override def logTransitionProbability(from: ModelFittingParameters, to: ModelFittingParameters): Double = {
    randomGaussianPoseProposal.logTransitionProbability(from, to)
  }
}