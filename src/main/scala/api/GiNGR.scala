package api

import breeze.linalg.{DenseMatrix, DenseVector}
import scalismo.common.PointId
import scalismo.geometry.{_3D, Landmark, Point}
import scalismo.mesh.TriangleMesh
import scalismo.registration.LandmarkRegistration
import scalismo.sampling.{DistributionEvaluator, ProposalGenerator, SymmetricTransitionRatio, TransitionProbability}
import scalismo.sampling.algorithms.MetropolisHastings
import scalismo.sampling.loggers.{BestSampleLogger, ChainStateLogger, ChainStateLoggerContainer}
import scalismo.statisticalmodel.{MultivariateNormalDistribution, PointDistributionModel}
import scalismo.transformations.{
  Scaling,
  TranslationAfterRotation,
  TranslationAfterRotationSpace3D,
  TranslationAfterScalingAfterRotation,
  TranslationAfterScalingAfterRotationSpace3D
}
import scalismo.utils.{Memoize, Random}
import scalismo.sampling.loggers.ChainStateLogger.implicits._
import scalismo.sampling.proposals.MixtureProposal
import scalismo.sampling.proposals.MixtureProposal.ProposalGeneratorWithTransition
import scalismo.sampling.proposals.MixtureProposal.implicits._

trait GlobalTranformationType
case object SimilarityTransforms extends GlobalTranformationType
case object RigidTransforms extends GlobalTranformationType
case object NoTransforms extends GlobalTranformationType

case class CorrespondencePairs(pairs: IndexedSeq[(PointId, Point[_3D])])

object CorrespondencePairs {
  def empty(): CorrespondencePairs = new CorrespondencePairs(IndexedSeq())
}

trait RegistrationState[T] {
  def iteration(): Int // Iterations left from current state
  def model(): PointDistributionModel[_3D, TriangleMesh] // Prior statistical mesh model
  def modelParameters(): DenseVector[Double] // parameters of the current fitting state in the model
  def modelLandmarks(): Option[Seq[Landmark[_3D]]] // Landmarks on the model
  def target(): TriangleMesh[_3D] // Target mesh
  def targetLandmarks(): Option[Seq[Landmark[_3D]]] // Landmarks on the target
  def fit(): TriangleMesh[_3D] // Current fit based on model parameters, global alignment and scaling
  def alignment(): TranslationAfterRotation[_3D] // Model translation and rotation
  def scaling(): Double = 1.0 // Model scaling
  def converged(): Boolean // Has the registration converged???
  def sigma2(): Double // Global uncertainty parameter
  def threshold: Double // Convergence threshold
  def globalTransformation(): GlobalTranformationType // Type of global transformation (none, rigid, similarity)
  def stepLength(): Double // Step length of a single registration step (0.0 to 1.0)
//  def probabilistic(): Boolean //
//  def nonRigidTransformation(): Boolean

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
  override val globalTransformation: GlobalTranformationType = RigidTransforms,
  override val stepLength: Double = 0.5
//  override val probabilistic: Boolean = false
//  override val nonRigidTransformation: Boolean = true
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
//  def apply(reference: TriangleMesh[_3D], target: TriangleMesh[_3D]): GeneralRegistrationState = {
//    val model: PointDistributionModel[_3D, TriangleMesh] = ???
//    apply(model, target, RigidTransforms)
//  }

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
    transform: GlobalTranformationType
//    nonRigidTransformation: Boolean
  ): GeneralRegistrationState = {
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
  def name: String
  private val cashedPosterior: Memoize[State, PointDistributionModel[_3D, TriangleMesh]] =
    Memoize(computePosterior, 3)
  val getCorrespondence: (State) => CorrespondencePairs
  val getUncertainty: (PointId, State) => MultivariateNormalDistribution

  private def computePosterior(current: State): PointDistributionModel[_3D, TriangleMesh] = {
    // Get correspondence between current model instance and the target
    val correspondences = getCorrespondence(current)
    // Add uncertainty to correspondence pairs
    val uncertainObservationsEstimated: IndexedSeq[(PointId, Point[_3D], MultivariateNormalDistribution)] = correspondences.pairs.map { pair =>
      val (pid, point) = pair
      val uncertainty = getUncertainty(pid, current)
      (pid, point, uncertainty)
    }
    // (Optional) Add manually assigned landmarks to observation sequence
    val uncertainObservations: IndexedSeq[(PointId, Point[_3D], MultivariateNormalDistribution)] =
      if (current.config.useLandmarkCorrespondence()) {
        uncertainObservationsEstimated ++ current.general.landmarkCorrespondences
      } else {
        uncertainObservationsEstimated
      }
    current.general.model.posterior(uncertainObservations)
  }

  def updateSigma2(current: State): State = {
    current
  }

  def update(current: State, probabilistic: Boolean)(implicit rnd: Random): State = {
    val posterior = cashedPosterior(current)
    val shapeproposal = if (!probabilistic) posterior.mean else posterior.sample()

    val newCoefficients = current.general.model.coefficients(shapeproposal)
    val currentShapeCoefficients = current.general.modelParameters
    val newShapeCoefficients = currentShapeCoefficients + (newCoefficients - currentShapeCoefficients) * current.general.stepLength

    val newshape = current.general.model.instance(newShapeCoefficients)

    // Need to scale proposal according to "step-length"
    val globalTransform: TranslationAfterScalingAfterRotation[_3D] = current.general.globalTransformation match {
      case SimilarityTransforms => similarityTransform(current.general.fit, newshape)
      case RigidTransforms => rigidTransform(current.general.fit, newshape)
      case _ => TranslationAfterScalingAfterRotationSpace3D(Point(0, 0, 0)).identityTransformation
    }
    val alignment: TranslationAfterRotation[_3D] = TranslationAfterRotation(globalTransform.translation, globalTransform.rotation)
    val transformedModel = current.general.model.transform(alignment)
    val alpha = transformedModel.coefficients(newshape)
    val fit = transformedModel.instance(alpha).transform(globalTransform.scaling)

    val general = current.general
      .updateIteration(current.general.iteration - 1)
      .updateFit(fit)
      .updateAlignment(alignment)
      .updateScaling(globalTransform.scaling.s)
      .updateModelParameters(alpha)
    updateSigma2(current.updateGeneral(general))
  }

  def rigidTransform(current: TriangleMesh[_3D], update: IndexedSeq[(PointId, Point[_3D])]): TranslationAfterScalingAfterRotation[_3D] = {
    val pairs = update.map(f => (current.pointSet.point(f._1), f._2))
    val t = LandmarkRegistration.rigid3DLandmarkRegistration(pairs, Point(0, 0, 0))
    TranslationAfterScalingAfterRotation(t.translation, Scaling(1.0), t.rotation)
  }

  def rigidTransform(current: TriangleMesh[_3D], update: TriangleMesh[_3D]): TranslationAfterScalingAfterRotation[_3D] = {
    val t = LandmarkRegistration.rigid3DLandmarkRegistration(current.pointSet.points.toSeq.zip(update.pointSet.points.toSeq), Point(0, 0, 0))
    TranslationAfterScalingAfterRotation(t.translation, Scaling(1.0), t.rotation)
  }

  def similarityTransform(current: TriangleMesh[_3D], update: IndexedSeq[(PointId, Point[_3D])]): TranslationAfterScalingAfterRotation[_3D] = {
    val pairs = update.map(f => (current.pointSet.point(f._1), f._2))
    LandmarkRegistration.similarity3DLandmarkRegistration(pairs, Point(0, 0, 0))
  }
  def similarityTransform(current: TriangleMesh[_3D], update: TriangleMesh[_3D]): TranslationAfterScalingAfterRotation[_3D] = {
    LandmarkRegistration.similarity3DLandmarkRegistration(current.pointSet.points.toSeq.zip(update.pointSet.points.toSeq), Point(0, 0, 0))
  }

  def instance(model: PointDistributionModel[_3D, TriangleMesh], state: GeneralRegistrationState): TriangleMesh[_3D] = {
    val scale = Scaling[_3D](state.scaling)
    model.instance(state.modelParameters).transform(state.alignment).transform(scale)
  }

  case class evaluatorWrapper(probabilistic: Boolean, evaluator: Option[DistributionEvaluator[State]]) extends DistributionEvaluator[State] {
    override def logValue(sample: State): Double = {
      if (!probabilistic) {
        0.0
      } else {
        evaluator.get.logValue(sample)
      }
    }
  }

  case class generatorWrapperDeterministic(generatedBy: String)(implicit rnd: Random) extends ProposalGenerator[State] with TransitionProbability[State] {
    override def propose(current: State): State = {
      println("Propose deterministic")
      update(current, probabilistic = false)
    }
    override def logTransitionProbability(from: State, to: State): Double = {
      0.0
    }
  }

  case class generatorWrapperStochastic(generatedBy: String)(implicit rnd: Random) extends ProposalGenerator[State] with TransitionProbability[State] {
    override def propose(current: State): State = {
      println("Propose stochastic")
      update(current, probabilistic = true)
    }

    override def logTransitionProbability(from: State, to: State): Double = {
      val posterior = cashedPosterior(from)

      val compensatedTo = from.general.modelParameters + ((to.general.modelParameters - from.general.modelParameters) / from.general.stepLength)
      val toMesh = from.general.model.instance(compensatedTo)

      val projectedTo = posterior.coefficients(toMesh)
      posterior.gp.logpdf(projectedTo)
    }
  }

  case class RandomShapeUpdateProposal(modelrank: Int, stdev: Double, generatedBy: String = "RandomShapeUpdateProposal")(implicit random: Random)
    extends ProposalGenerator[State] with SymmetricTransitionRatio[State] with TransitionProbability[State] {

    private val perturbationDistr = new MultivariateNormalDistribution(DenseVector.zeros(modelrank), DenseMatrix.eye[Double](modelrank) * stdev * stdev)

    override def propose(theta: State): State = {
      println("Random propose")
      val currentCoeffs = theta.general.modelParameters
      val updatedCoeffs = currentCoeffs + perturbationDistr.sample
      theta.updateGeneral(theta.general.updateIteration(theta.general.iteration - 1).updateModelParameters(updatedCoeffs))
    }

    override def logTransitionProbability(from: State, to: State): Double = {
      val residual = to.general.modelParameters - from.general.modelParameters
      perturbationDistr.logpdf(residual)
    }
  }

  def generatorCombined(probabilistic: Boolean, modelrank: Int, mixing: Option[ProposalGenerator[State] with TransitionProbability[State]])(implicit
    rnd: Random): ProposalGenerator[State] with TransitionProbability[State] = {
    if (!probabilistic) generatorWrapperDeterministic(name)
    else {
      val mix = mixing.getOrElse {
        MixtureProposal(
          0.05 *: RandomShapeUpdateProposal(modelrank, 0.1, generatedBy = "RandomShape-0.1") +
            0.05 *: RandomShapeUpdateProposal(modelrank, 0.01, generatedBy = "RandomShape-0.01") +
            0.05 *: RandomShapeUpdateProposal(modelrank, 0.001, generatedBy = "RandomShape-0.001")
        )
      }
      val informedGenerator = generatorWrapperStochastic(name)
      val totalMix = mix.map(_._1).sum
      println(s"Mixing: ${totalMix}, other ${1.0 - totalMix}")
      MixtureProposal(0.2 *: mix + 0.8 *: informedGenerator)
    }
  }

  case class emptyLogger() extends ChainStateLogger[State] {
    override def logState(sample: State): Unit = {}
  }

  case class AcceptAllEvaluator() extends DistributionEvaluator[State] {
    override def logValue(sample: State): Double = 0.0
  }

  def run(
    initialState: State,
    callBack: ChainStateLogger[State] = emptyLogger(),
    evaluatorInit: Option[DistributionEvaluator[State]] = None,
    generatorMixing: Option[ProposalGenerator[State] with TransitionProbability[State]] = None,
    probabilistic: Boolean = false // false = deterministic registration. true = probabilistic registration
  )(implicit rnd: Random): State = {
    // Check that evaluator is available if probabilistic setting!
    require(true)
    val evaluator = Some(evaluatorInit.getOrElse(AcceptAllEvaluator()))
    val registrationEvaluator = evaluatorWrapper(probabilistic, evaluator)
    val registrationGenerator = generatorCombined(probabilistic, initialState.general.model.rank, generatorMixing)
    val bestSampleLogger = BestSampleLogger[State](registrationEvaluator)
    val loggers = ChainStateLoggerContainer(Seq(bestSampleLogger, callBack))
    val mhChain = MetropolisHastings[State](registrationGenerator, registrationEvaluator)

    val states = mhChain.iterator(initialState).loggedWith(loggers)
    states.dropWhile(state => !state.general.converged && state.general.iteration > 0).next()
    val fit = bestSampleLogger.currentBestSample().get
    fit
  }
}
