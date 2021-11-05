package api.sampling.generators

import api.{GingrRegistrationState, ShapeParameters}
import scalismo.sampling.{ProposalGenerator, TransitionProbability, TransitionRatio}
import scalismo.utils.Random

case class RandomShapeUpdateProposal[State <: GingrRegistrationState[State]](stdev: Double, generatedBy: String = "RandomShapeUpdateProposal")(implicit random: Random)
  extends ProposalGenerator[State] with TransitionRatio[State] with TransitionProbability[State] {

  private val generator = GaussianDenseVectorProposal(stdev)

  override def propose(theta: State): State = {
    val currentCoeffs = theta.general.modelParameters.shape.parameters
    val updatedCoeffs = generator.propose(currentCoeffs)
    theta.updateGeneral(theta.general.updateShapeParameters(ShapeParameters(updatedCoeffs)).updateGeneratedBy(generatedBy))
  }

  override def logTransitionProbability(from: State, to: State): Double = {
    if (to.general.modelParameters.copy(shape = from.general.modelParameters.shape) != from.general.modelParameters) {
      Double.NegativeInfinity
    } else {
      generator.logTransitionProbability(from.general.modelParameters.shape.parameters, to.general.modelParameters.shape.parameters)
    }
  }
}
