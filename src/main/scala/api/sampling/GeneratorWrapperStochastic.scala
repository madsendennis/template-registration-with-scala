package api.sampling

import api.GingrRegistrationState
import scalismo.geometry._3D
import scalismo.mesh.TriangleMesh
import scalismo.sampling.{ProposalGenerator, TransitionProbability}
import scalismo.statisticalmodel.PointDistributionModel
import scalismo.utils.{Memoize, Random}

case class GeneratorWrapperStochastic[State <: GingrRegistrationState[State]](update: (State,Boolean) => State,cashedPosterior: Memoize[State, PointDistributionModel[_3D, TriangleMesh]], generatedBy: String)(implicit rnd: Random) extends ProposalGenerator[State] with TransitionProbability[State] {
  override def propose(current: State): State = {
    println("Propose stochastic")
    update(current, true)
  }

  override def logTransitionProbability(from: State, to: State): Double = {
    val posterior = cashedPosterior(from)

    val compensatedTo = from.general.modelParameters + ((to.general.modelParameters - from.general.modelParameters) / from.general.stepLength)
    val toMesh = from.general.model.instance(compensatedTo)

    val projectedTo = posterior.coefficients(toMesh)
    posterior.gp.logpdf(projectedTo)
  }
}
