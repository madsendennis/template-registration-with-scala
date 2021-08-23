package api

import breeze.linalg.DenseVector
import scalismo.common.PointId
import scalismo.geometry.{Landmark, Point, _3D}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.{MultivariateNormalDistribution, PointDistributionModel, StatisticalMeshModel}
import scalismo.transformations.RigidTransformation

case class CorrespondencePairs(pairs: IndexedSeq[(PointId, Point[_3D])])

trait GingrConfig[T <: GingrRegistrationState[T]] {
  def maxIterations(): Int

  def converged: (T, T) => Boolean
}

trait GingrRegistrationState[T] {
  val iteration: Int

  /** initial prior model */
  def model(): PointDistributionModel[_3D, TriangleMesh]

  /** parameters of the current fitting state in the initial prior model */
  def modelParameters(): DenseVector[Double]

  def target(): TriangleMesh[_3D]

  def fit(): TriangleMesh[_3D]

  def rigidAlignment(): RigidTransformation[_3D]

  def scaling(): Double = 1.0

  def converged(): Boolean

  /**
    * Updates the current state with the new fit.
    *
    * @param next The newly calculated shape / fit.
    */
  private[api] def updateFit(next: TriangleMesh[_3D]): T
}

trait GingrAlgorithm[State <: GingrRegistrationState[State]] {
  def initialize(): State

  val getCorrespondence: (State) => CorrespondencePairs
  val getUncertainty: (PointId, State) => MultivariateNormalDistribution

  def update(current: State): State = {
    val currentFit = current.fit()
    val correspondences = getCorrespondence(current)
    val uncertainObservations = correspondences.pairs.map { pair =>
      val (pid, point) = pair
      val uncertainty = getUncertainty(pid, current)
      (pid, point, uncertainty)
    }
    val posterior = current.model.posterior(uncertainObservations)
    current.updateFit(posterior.mean)
  }

  def run(
           target: TriangleMesh[_3D],
           targetLandmarks: Option[Seq[Landmark[_3D]]],
           model: PointDistributionModel[_3D, TriangleMesh],
           modelLandmarks: Option[Seq[Landmark[_3D]]]
         ): State = {
    val initialState: State = initialize()
    runFromState(target, targetLandmarks, model, modelLandmarks, initialState)
  }

  def runFromState(
                    target: TriangleMesh[_3D],
                    targetLandmarks: Option[Seq[Landmark[_3D]]],
                    model: PointDistributionModel[_3D, TriangleMesh],
                    modelLandmarks: Option[Seq[Landmark[_3D]]],
                    initialState: State
                  ): State = {

    val registration: Iterator[State] = Iterator.iterate(initialState) { current =>
      val next = update(current)
      next
    }

    val states: Iterator[State] = registration.take(100)
    val fit: State = states.dropWhile(state =>
      !state.converged() && state.iteration > 0
    ).next()
    fit
  }
}