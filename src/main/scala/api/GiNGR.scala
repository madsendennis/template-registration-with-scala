package api

import breeze.linalg.{DenseMatrix, DenseVector}
import scalismo.common.PointId
import scalismo.geometry.{_3D, Landmark, Point}
import scalismo.mesh.TriangleMesh
import scalismo.registration.LandmarkRegistration
import scalismo.statisticalmodel.{MultivariateNormalDistribution, PointDistributionModel}
import scalismo.transformations.{
  Scaling,
  TranslationAfterRotation,
  TranslationAfterRotationSpace3D,
  TranslationAfterScalingAfterRotation,
  TranslationAfterScalingAfterRotationSpace3D
}

trait GlobalTranformationType

case object SimilarityTransforms extends GlobalTranformationType
case object RigidTransforms extends GlobalTranformationType
case object NoTransforms extends GlobalTranformationType

case class CorrespondencePairs(pairs: IndexedSeq[(PointId, Point[_3D])])

object CorrespondencePairs {
  def empty(): CorrespondencePairs = new CorrespondencePairs(IndexedSeq())
}

trait RegistrationState[T] {
  def iteration(): Int
  /** initial prior model */
  def model(): PointDistributionModel[_3D, TriangleMesh]
  /** parameters of the current fitting state in the initial prior model */
  def modelParameters(): DenseVector[Double]
  def modelLandmarks(): Option[Seq[Landmark[_3D]]]
  def target(): TriangleMesh[_3D]
  def targetLandmarks(): Option[Seq[Landmark[_3D]]]
  def fit(): TriangleMesh[_3D]
  def alignment(): TranslationAfterRotation[_3D]
  def scaling(): Double = 1.0
  def converged(): Boolean
  def sigma2(): Double
  def threshold: Double
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

case class GeneralRegistrationState(
  override val model: PointDistributionModel[_3D, TriangleMesh],
  override val modelParameters: DenseVector[Double],
  override val modelLandmarks: Option[Seq[Landmark[_3D]]] = None,
  override val target: TriangleMesh[_3D],
  override val targetLandmarks: Option[Seq[Landmark[_3D]]] = None,
  override val fit: TriangleMesh[_3D],
  override val alignment: TranslationAfterRotation[_3D],
  override val scaling: Double = 1.0,
  override val converged: Boolean = false,
  override val sigma2: Double = 1.0,
  override val threshold: Double = 1e-10,
  override val iteration: Int = 0,
  override val globalTransformation: GlobalTranformationType = RigidTransforms
) extends RegistrationState[GeneralRegistrationState] {

  /** Updates the current state with the new fit.
    *
    * @param next
    *   The newly calculated shape / fit.
    */
  override def updateFit(next: TriangleMesh[_3D]): GeneralRegistrationState = this.copy(fit = next)
  override private[api] def updateAlignment(next: TranslationAfterRotation[_3D]): GeneralRegistrationState = this.copy(alignment = next)
  override private[api] def updateScaling(next: Double): GeneralRegistrationState = this.copy(scaling = next)
  override private[api] def updateModelParameters(next: DenseVector[Double]): GeneralRegistrationState = this.copy(modelParameters = next)
  override private[api] def updateIteration(next: Int): GeneralRegistrationState = this.copy(iteration = next)

  lazy val landmarkCorrespondences: IndexedSeq[(PointId, Point[_3D], MultivariateNormalDistribution)] = {
    if (modelLandmarks.nonEmpty && targetLandmarks.nonEmpty) {
      val m = modelLandmarks.get
      val t = targetLandmarks.get
      val commonLmNames = m.map(_.id) intersect t.map(_.id)
      commonLmNames.map { name =>
        val mPoint = m.find(_.id == name).get
        val tPoint = t.find(_.id == name).get
        (
          model.reference.pointSet.findClosestPoint(mPoint.point).id,
          tPoint.point,
          mPoint.uncertainty.getOrElse(MultivariateNormalDistribution(DenseVector.zeros[Double](3), DenseMatrix.eye[Double](3)))
        )
      }.toIndexedSeq
    } else {
      IndexedSeq()
    }
  }
}

trait GingrConfig {
  def maxIterations(): Int
  def converged: (GeneralRegistrationState, GeneralRegistrationState) => Boolean
  def useLandmarkCorrespondence(): Boolean
}

trait GingrRegistrationState[State] {
  def general: GeneralRegistrationState
  def config: GingrConfig
  private[api] def updateGeneral(update: GeneralRegistrationState): State
}

object GeneralRegistrationState {
  def apply(model: PointDistributionModel[_3D, TriangleMesh], target: TriangleMesh[_3D]): GeneralRegistrationState = {
    apply(model, target, RigidTransforms)
  }

  def apply(model: PointDistributionModel[_3D, TriangleMesh], target: TriangleMesh[_3D], transform: GlobalTranformationType): GeneralRegistrationState = {
    apply(model, Seq(), target, Seq(), transform)
  }

  def apply(
    model: PointDistributionModel[_3D, TriangleMesh],
    modelLandmarks: Seq[Landmark[_3D]],
    target: TriangleMesh[_3D],
    targetLandmarks: Seq[Landmark[_3D]]
  ): GeneralRegistrationState = {
    apply(model, modelLandmarks, target, targetLandmarks, RigidTransforms)
  }

  def apply(
    model: PointDistributionModel[_3D, TriangleMesh],
    modelLandmarks: Seq[Landmark[_3D]],
    target: TriangleMesh[_3D],
    targetLandmarks: Seq[Landmark[_3D]],
    transform: GlobalTranformationType): GeneralRegistrationState = {
    val initial =
      new GeneralRegistrationState(
        model = model,
        modelParameters = DenseVector.zeros[Double](model.rank),
        modelLandmarks = Option(modelLandmarks),
        target = target,
        targetLandmarks = Option(targetLandmarks),
        fit = model.mean,
        alignment = TranslationAfterRotationSpace3D(Point(0, 0, 0)).identityTransformation,
        globalTransformation = transform
      )
    initial
  }
}

trait GingrAlgorithm[State <: GingrRegistrationState[State]] {
  val getCorrespondence: (State) => CorrespondencePairs
  val getUncertainty: (PointId, State) => MultivariateNormalDistribution

  def updateSigma2(current: State): State = {
    current
  }

  def update(current: State): State = {
    val correspondences = getCorrespondence(current)
    val uncertainObservationsEstimated: IndexedSeq[(PointId, Point[_3D], MultivariateNormalDistribution)] = correspondences.pairs.map { pair =>
      val (pid, point) = pair
      val uncertainty = getUncertainty(pid, current)
      (pid, point, uncertainty)
    }
    val uncertainObservations =
      if (current.config.useLandmarkCorrespondence()) uncertainObservationsEstimated ++ current.general.landmarkCorrespondences else uncertainObservationsEstimated
    val posterior = current.general.model.posterior(uncertainObservations)
    val globalTranform: TranslationAfterScalingAfterRotation[_3D] = current.general.globalTransformation match {
      case SimilarityTransforms => similarityTransform(current.general.fit, posterior.mean)
      case RigidTransforms => rigidTransform(current.general.fit, posterior.mean)
      case _ => TranslationAfterScalingAfterRotationSpace3D(Point(0, 0, 0)).identityTransformation
    }
    val alignment = TranslationAfterRotation(globalTranform.translation, globalTranform.rotation)
    val transformedModel = current.general.model.transform(alignment)
    val alpha = transformedModel.coefficients(posterior.mean)
    val fit = transformedModel.instance(alpha).transform(globalTranform.scaling)

    val general = current.general
      .updateIteration(current.general.iteration - 1)
      .updateFit(fit)
      .updateAlignment(alignment)
      .updateScaling(globalTranform.scaling.s)
      .updateModelParameters(alpha)
    updateSigma2(current.updateGeneral(general))
  }

  def rigidTransform(current: TriangleMesh[_3D], update: TriangleMesh[_3D]): TranslationAfterScalingAfterRotation[_3D] = {
    val t = LandmarkRegistration.rigid3DLandmarkRegistration(current.pointSet.points.toSeq.zip(update.pointSet.points.toSeq), Point(0, 0, 0))
    TranslationAfterScalingAfterRotation(t.translation, Scaling(1.0), t.rotation)
  }

  def similarityTransform(current: TriangleMesh[_3D], update: TriangleMesh[_3D]): TranslationAfterScalingAfterRotation[_3D] = {
    LandmarkRegistration.similarity3DLandmarkRegistration(current.pointSet.points.toSeq.zip(update.pointSet.points.toSeq), Point(0, 0, 0))
  }

  def instance(model: PointDistributionModel[_3D, TriangleMesh], state: GeneralRegistrationState): TriangleMesh[_3D] = {
    val scale = Scaling[_3D](state.scaling)
    model.instance(state.modelParameters).transform(state.alignment).transform(scale)
  }

  def run(
    initialState: State,
    callBack: State => Unit = _ => None
  ): State = {
    val registration: Iterator[State] = Iterator.iterate(initialState) { current =>
      val next = update(current)
      callBack(next)
      next
    }
    val states: Iterator[State] = registration.take(10000)
    val fit: State = states.dropWhile(state => !state.general.converged && state.general.iteration > 0).next()
    fit
  }
}
