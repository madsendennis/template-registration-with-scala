package api.sampling

import api.GingrRegistrationState
import scalismo.sampling.DistributionEvaluator

case class EvaluatorWrapper[State <: GingrRegistrationState[State]](probabilistic: Boolean, evaluator: Option[DistributionEvaluator[State]]) extends DistributionEvaluator[State] {
  override def logValue(sample: State): Double = {
    if (!probabilistic) {
      0.0
    } else {
      evaluator.get.logValue(sample)
    }
  }
}
