package api.sampling

import api.GingrRegistrationState
import scalismo.sampling.{ProposalGenerator, TransitionProbability, TransitionRatio}
import scalismo.utils.Random

case class RandomShapeUpdateProposal[State <: GingrRegistrationState[State]](stdev: Double, generatedBy: String = "RandomShapeUpdateProposal")(implicit random: Random)
  extends ProposalGenerator[State] with TransitionRatio[State] with TransitionProbability[State] {

  private val generator = GaussianDenseVectorProposal(stdev)

  override def propose(theta: State): State = {
    println("Random propose")
    val currentCoeffs = theta.general.modelParameters
    val updatedCoeffs = generator.propose(currentCoeffs)
    theta.updateGeneral(theta.general.updateModelParameters(updatedCoeffs))
  }

  override def logTransitionProbability(from: State, to: State): Double = {
    generator.logTransitionProbability(from.general.modelParameters,to.general.modelParameters)
  }
}
