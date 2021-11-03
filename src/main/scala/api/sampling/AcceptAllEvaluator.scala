package api.sampling

import api.GingrRegistrationState
import scalismo.sampling.DistributionEvaluator

case class AcceptAllEvaluator[State <: GingrRegistrationState[State]]() extends DistributionEvaluator[State] {
    override def logValue(sample: State): Double = 0.0
  }
