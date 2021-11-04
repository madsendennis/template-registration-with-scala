package api.sampling

import api.GingrRegistrationState
import api.sampling.generators.RandomShapeUpdateProposal
import scalismo.sampling.{ProposalGenerator, TransitionProbability}
import scalismo.sampling.proposals.MixtureProposal
import scalismo.utils.Random
import scalismo.sampling.proposals.MixtureProposal.implicits._
import scalismo.sampling.proposals.MixtureProposal.ProposalGeneratorWithTransition

case class Generator[State <: GingrRegistrationState[State]](implicit rnd: Random) {

  def RandomShape(steps: Seq[Double] = Seq(1.0, 0.1, 0.01)): ProposalGenerator[State] with TransitionProbability[State] = {
    val gen = steps.map { d => (1.0 / steps.length.toDouble, RandomShapeUpdateProposal[State](d, generatedBy = s"RandomShape-${d}")) }
    MixtureProposal.fromProposalsWithTransition(gen: _*)
  }
}
