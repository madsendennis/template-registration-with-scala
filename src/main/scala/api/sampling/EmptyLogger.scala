package api.sampling

import api.GingrRegistrationState
import scalismo.sampling.loggers.ChainStateLogger


case class EmptyLogger[State <: GingrRegistrationState[State]]() extends ChainStateLogger[State] {
    override def logState(sample: State): Unit = {}
  }

