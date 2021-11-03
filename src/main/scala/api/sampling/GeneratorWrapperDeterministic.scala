package api.sampling

import api.GingrRegistrationState
import scalismo.sampling.{ProposalGenerator, TransitionProbability}
import scalismo.utils.Random

case class GeneratorWrapperDeterministic[State <: GingrRegistrationState[State]](update: (State,Boolean) => State, generatedBy: String)(implicit rnd: Random) extends ProposalGenerator[State] with TransitionProbability[State] {
  override def propose(current: State): State = {
    println("Propose deterministic")
    update(current, false)
  }
  override def logTransitionProbability(from: State, to: State): Double = {
    0.0
  }
}
