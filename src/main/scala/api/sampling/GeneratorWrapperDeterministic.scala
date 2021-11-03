package api.sampling

import api.GingrRegistrationState
import scalismo.sampling.{ProposalGenerator, TransitionProbability}
import scalismo.utils.Random

case class GeneratorWrapperDeterministic[State <: GingrRegistrationState[State]](update: (State,Boolean) => State, generatedBy: String = "Deterministic")(implicit rnd: Random) extends ProposalGenerator[State] with TransitionProbability[State] {
  override def propose(current: State): State = {
    update(current, false)
  }
  override def logTransitionProbability(from: State, to: State): Double = {
    0.0
  }
}
