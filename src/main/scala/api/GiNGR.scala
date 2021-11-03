package api

import api.sampling.{EvaluatorWrapper, GeneratorWrapperDeterministic, GeneratorWrapperStochastic, RandomShapeUpdateProposal}
import breeze.linalg.{DenseMatrix, DenseVector}
import scalismo.common.PointId
import scalismo.geometry.{Landmark, Point, _3D}
import scalismo.mesh.TriangleMesh
import scalismo.registration.LandmarkRegistration
import scalismo.sampling.algorithms.MetropolisHastings
import scalismo.sampling.loggers.ChainStateLogger.implicits._
import scalismo.sampling.loggers.{BestSampleLogger, ChainStateLogger, ChainStateLoggerContainer}
import scalismo.sampling.proposals.MixtureProposal
import scalismo.sampling.proposals.MixtureProposal.implicits._
import scalismo.sampling.{DistributionEvaluator, ProposalGenerator, TransitionProbability}
import scalismo.statisticalmodel.{MultivariateNormalDistribution, PointDistributionModel}
import scalismo.transformations._
import scalismo.utils.{Memoize, Random}

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
  val getCorrespondence: (State) => CorrespondencePairs
  val getUncertainty: (PointId, State) => MultivariateNormalDistribution

  private def computePosterior(current: State): PointDistributionModel[_3D, TriangleMesh] = {
    val correspondences = getCorrespondence(current)
    val correspondencesWithUncertainty = correspondences.pairs.map { pair =>
      val (pid, point) = pair
      val uncertainty = getUncertainty(pid, current)
      (pid, point, uncertainty)
    }

    val observationsWithUncertainty = if (current.config.useLandmarkCorrespondence()) {
      // note: I would propose to remove the estimated correspondences for given landmarks
      val landmarksToUse = current.general.landmarkCorrespondences.map(_._1).toSet
      val filteredCorrespondencesWithUncertainty = correspondencesWithUncertainty.filter{ case (pid,_,_) => !landmarksToUse.contains(pid)}
      filteredCorrespondencesWithUncertainty ++ current.general.landmarkCorrespondences
    } else {
      correspondencesWithUncertainty
    }
    current.general.model.posterior(observationsWithUncertainty)
  }
  private val cashedPosterior = Memoize(computePosterior, 3)


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

  def generatorCombined(probabilistic: Boolean, modelrank: Int, mixing: Option[ProposalGenerator[State] with TransitionProbability[State]])(implicit
                                                                                                                                            rnd: Random): ProposalGenerator[State] with TransitionProbability[State] = {
    if (!probabilistic) GeneratorWrapperDeterministic(update, name)
    else {
      val mix = mixing.getOrElse {
        MixtureProposal(
          0.05 *: RandomShapeUpdateProposal(modelrank, 0.1, generatedBy = "RandomShape-0.1") +
            0.05 *: RandomShapeUpdateProposal(modelrank, 0.01, generatedBy = "RandomShape-0.01") +
            0.05 *: RandomShapeUpdateProposal(modelrank, 0.001, generatedBy = "RandomShape-0.001")
        )
      }
      val informedGenerator = GeneratorWrapperStochastic(update, cashedPosterior, name)
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

  /**
    * Runs the actual registration with the provided configuration through the passed parameters.
    *
    * @param initialState State from which the registration is started.
    * @param callBack Logger triggered every iteration (for sub sampling the logging, see ChainStateLogger.subSampled() ).
    * @param evaluatorInit
    * @param generatorMixing
    * @param probabilistic Flag which switches between probabilistic (true) and deterministic (false)
    * @param rnd Implicit random number generator.
    * @return Returns the best sample of the chain given the evaluator..
    */
  def run(
           initialState: State,
           callBack: ChainStateLogger[State] = emptyLogger(),
           evaluatorInit: Option[DistributionEvaluator[State]] = None,
           generatorMixing: Option[ProposalGenerator[State] with TransitionProbability[State]] = None,
           probabilistic: Boolean = false
         )(implicit rnd: Random): State = {
    // Check that evaluator is available if probabilistic setting!
    require(true)
    val evaluator = Some(evaluatorInit.getOrElse(AcceptAllEvaluator()))
    val registrationEvaluator = EvaluatorWrapper(probabilistic, evaluator)
    val registrationGenerator = generatorCombined(probabilistic, initialState.general.model.rank, generatorMixing)
    val bestSampleLogger = BestSampleLogger[State](registrationEvaluator)
    val loggers = ChainStateLoggerContainer(Seq(bestSampleLogger, callBack))
    val mhChain = MetropolisHastings[State](registrationGenerator, registrationEvaluator)

    val states = mhChain.iterator(initialState).loggedWith(loggers)

    // we need to query if there is a next element, otherwise due to laziness the chain is not calculated
    states.take(initialState.general.iteration).dropWhile(state => !state.general.converged).hasNext

    val fit = bestSampleLogger.currentBestSample().get
    fit
  }
}
