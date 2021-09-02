package api

import breeze.linalg.DenseVector
import scalismo.common.PointId
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.geometry.{_3D, Landmark, Point}
import scalismo.mesh.TriangleMesh
import scalismo.registration.LandmarkRegistration
import scalismo.statisticalmodel.{MultivariateNormalDistribution, PointDistributionModel, StatisticalMeshModel}
import scalismo.transformations.{RigidTransformation, Scaling, TranslationAfterRotation, TranslationAfterScalingAfterRotation, TranslationAfterScalingAfterRotationSpace3D}

trait GlobalTranformationType

case object SimilarityTransforms extends GlobalTranformationType

case object RigidTransforms extends GlobalTranformationType

case object NoTransforms extends GlobalTranformationType

case class CorrespondencePairs(pairs: IndexedSeq[(PointId, Point[_3D])])

trait GingrConfig[T <: GingrRegistrationState[T]] {
  def maxIterations(): Int

  def converged: (T, T) => Boolean
}

trait GingrRegistrationState[T] {
  def iteration(): Int

  /** initial prior model */
  def model(): PointDistributionModel[_3D, TriangleMesh]

  /** parameters of the current fitting state in the initial prior model */
  def modelParameters(): DenseVector[Double]

  def target(): TriangleMesh[_3D]

  def fit(): TriangleMesh[_3D]

  def alignment(): TranslationAfterRotation[_3D]

  def scaling(): Double = 1.0

  def converged(): Boolean

  def sigma2(): Double

  def globalTransformation(): GlobalTranformationType

  /** Updates the current state with the new fit.
    *
    * @param next
    *   The newly calculated shape / fit.
    */
  private[api] def updateFit(next: TriangleMesh[_3D]): T

  private[api] def updateAlignment(next: TranslationAfterRotation[_3D]): T

  private[api] def updateScaling(next: Double): T

  private[api] def updateModelParameters(next: DenseVector[Double]): T

  private[api] def updateIteration(next: Int): T
}

trait GingrAlgorithm[State <: GingrRegistrationState[State]] {
  def initialize(): State

  val getCorrespondence: (State) => CorrespondencePairs
  val getUncertainty: (PointId, State) => MultivariateNormalDistribution

  def updateSigma2(current: State): State = {
    current
  }

  def update(current: State): State = {
    println(s"Iteration: ${current.iteration()}, sigma2: ${current.sigma2()}")
    val correspondences = getCorrespondence(current)
    val uncertainObservations = correspondences.pairs.map { pair =>
      val (pid, point) = pair
      val uncertainty = getUncertainty(pid, current)
      (pid, point, uncertainty)
    }
    val posterior = current.model().posterior(uncertainObservations)
    val globalTranform: TranslationAfterScalingAfterRotation[_3D] = current.globalTransformation() match {
      case SimilarityTransforms => similarityTransform(current.fit(), posterior.mean)
      case RigidTransforms => rigidTransform(current.fit(), posterior.mean)
      case _ => TranslationAfterScalingAfterRotationSpace3D(Point(0, 0, 0)).identityTransformation
    }
    val alignment = TranslationAfterRotation(globalTranform.translation, globalTranform.rotation)
    val transformedModel = current.model().transform(alignment)
    val alpha = transformedModel.coefficients(posterior.mean)
    val fit = transformedModel.instance(alpha).transform(globalTranform.scaling)

    val updated = current
      .updateIteration(current.iteration() - 1)
      .updateFit(fit)
      .updateAlignment(alignment)
      .updateScaling(globalTranform.scaling.s)
      .updateModelParameters(alpha)
    updateSigma2(updated)
  }

  def run(
    target: TriangleMesh[_3D],
    targetLandmarks: Option[Seq[Landmark[_3D]]],
    model: PointDistributionModel[_3D, TriangleMesh],
    modelLandmarks: Option[Seq[Landmark[_3D]]],
    callBack: State => Unit = _ => None
  ): State = {
    val initialState: State = initialize()
    runFromState(target, targetLandmarks, model, modelLandmarks, callBack, initialState)
  }

  def rigidTransform(current: TriangleMesh[_3D], update: TriangleMesh[_3D]): TranslationAfterScalingAfterRotation[_3D] = {
    val t = LandmarkRegistration.rigid3DLandmarkRegistration(current.pointSet.points.toSeq.zip(update.pointSet.points.toSeq), Point(0, 0, 0))
    TranslationAfterScalingAfterRotation(t.translation, Scaling(1.0), t.rotation)
  }

  def similarityTransform(current: TriangleMesh[_3D], update: TriangleMesh[_3D]): TranslationAfterScalingAfterRotation[_3D] = {
    LandmarkRegistration.similarity3DLandmarkRegistration(current.pointSet.points.toSeq.zip(update.pointSet.points.toSeq), Point(0, 0, 0))
  }

  def runFromState(
    target: TriangleMesh[_3D],
    targetLandmarks: Option[Seq[Landmark[_3D]]],
    model: PointDistributionModel[_3D, TriangleMesh],
    modelLandmarks: Option[Seq[Landmark[_3D]]],
    callBack: State => Unit = _ => None,
    initialState: State
  ): State = {
    val registration: Iterator[State] = Iterator.iterate(initialState) { current =>
      val next = update(current)
      callBack(next)
      next
    }

    val states: Iterator[State] = registration.take(100)
    val fit: State = states.dropWhile(state => !state.converged() && state.iteration > 0).next()
    fit
  }
}
