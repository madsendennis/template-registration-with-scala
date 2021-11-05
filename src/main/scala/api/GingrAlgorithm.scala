package api

import api.sampling.{AcceptAll, Evaluator, Generator}
import api.sampling.evaluators.EvaluatorWrapper
import api.sampling.loggers.{EmptyAcceptRejectLogger, EmptyChainStateLogger}
import api.sampling.generators.{GeneratorWrapperDeterministic, GeneratorWrapperStochastic, RandomShapeUpdateProposal}
import scalismo.common.PointId
import scalismo.geometry.{_3D, Point}
import scalismo.mesh.TriangleMesh
import scalismo.registration.LandmarkRegistration
import scalismo.sampling.algorithms.MetropolisHastings
import scalismo.sampling.loggers.{AcceptRejectLogger, BestSampleLogger, ChainStateLogger, ChainStateLoggerContainer}
import scalismo.sampling.loggers.ChainStateLogger.implicits._
import scalismo.sampling.proposals.MixtureProposal
import scalismo.sampling.proposals.MixtureProposal.implicits._
import scalismo.sampling.{ProposalGenerator, TransitionProbability}
import scalismo.statisticalmodel.{MultivariateNormalDistribution, PointDistributionModel}
import scalismo.transformations.{Scaling, TranslationAfterRotation, TranslationAfterScalingAfterRotation, TranslationAfterScalingAfterRotationSpace3D}
import scalismo.utils.{Memoize, Random}

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
      val filteredCorrespondencesWithUncertainty = correspondencesWithUncertainty.filter { case (pid, _, _) => !landmarksToUse.contains(pid) }
      filteredCorrespondencesWithUncertainty ++ current.general.landmarkCorrespondences
    } else {
      correspondencesWithUncertainty
    }
    current.general.model.posterior(observationsWithUncertainty)
  }
  private val cashedPosterior = Memoize(computePosterior, 10)

  def updateSigma2(current: State): State = {
    current
  }

  def update(current: State, probabilistic: Boolean)(implicit rnd: Random): State = {
    val posterior = cashedPosterior(current)
    val shapeproposal = if (!probabilistic) posterior.mean else posterior.sample()

    val newCoefficients = current.general.model.coefficients(shapeproposal)
    val currentShapeCoefficients = current.general.modelParameters.shape.parameters
    val newShapeCoefficients = currentShapeCoefficients + (newCoefficients - currentShapeCoefficients) * current.general.stepLength

    val newshape = current.general.model.instance(newShapeCoefficients)

    // Need to scale proposal according to "step-length"
    val globalTransform: TranslationAfterScalingAfterRotation[_3D] = current.general.globalTransformation match {
      case SimilarityTransforms => estimateSimilarityTransform(current.general.fit, newshape)
      case RigidTransforms => estimateRigidTransform(current.general.fit, newshape)
      case _ => TranslationAfterScalingAfterRotationSpace3D(Point(0, 0, 0)).identityTransformation
    }
    val alignment: TranslationAfterRotation[_3D] = TranslationAfterRotation(globalTransform.translation, globalTransform.rotation)
    val transformedModel = current.general.model.transform(alignment)
    val alpha = transformedModel.coefficients(newshape)
    val fit = transformedModel.instance(alpha).transform(globalTransform.scaling)
    val general = current.general
      .updateFit(fit)
      .updateTranslation(alignment.translation.t)
      .updateRotation(alignment.rotation)
      .updateScaling(ScaleParameter(globalTransform.scaling.s))
      .updateShapeParameters(ShapeParameters(alpha))
    updateSigma2(current.updateGeneral(general))
  }

  def estimateRigidTransform(current: TriangleMesh[_3D], update: IndexedSeq[(PointId, Point[_3D])]): TranslationAfterScalingAfterRotation[_3D] = {
    val pairs = update.map(f => (current.pointSet.point(f._1), f._2))
    val t = LandmarkRegistration.rigid3DLandmarkRegistration(pairs, Point(0, 0, 0))
    TranslationAfterScalingAfterRotation(t.translation, Scaling(1.0), t.rotation)
  }

  def estimateRigidTransform(current: TriangleMesh[_3D], update: TriangleMesh[_3D]): TranslationAfterScalingAfterRotation[_3D] = {
    val t = LandmarkRegistration.rigid3DLandmarkRegistration(current.pointSet.points.toSeq.zip(update.pointSet.points.toSeq), Point(0, 0, 0))
    TranslationAfterScalingAfterRotation(t.translation, Scaling(1.0), t.rotation)
  }

  def estimateSimilarityTransform(current: TriangleMesh[_3D], update: IndexedSeq[(PointId, Point[_3D])]): TranslationAfterScalingAfterRotation[_3D] = {
    val pairs = update.map(f => (current.pointSet.point(f._1), f._2))
    LandmarkRegistration.similarity3DLandmarkRegistration(pairs, Point(0, 0, 0))
  }
  def estimateSimilarityTransform(current: TriangleMesh[_3D], update: TriangleMesh[_3D]): TranslationAfterScalingAfterRotation[_3D] = {
    LandmarkRegistration.similarity3DLandmarkRegistration(current.pointSet.points.toSeq.zip(update.pointSet.points.toSeq), Point(0, 0, 0))
  }

  def instance(model: PointDistributionModel[_3D, TriangleMesh], state: GeneralRegistrationState): TriangleMesh[_3D] = {
    model.instance(state.modelParameters.shape.parameters).transform(state.similarityTransform)
  }

  def generatorCombined(probabilistic: Boolean, mixing: Option[ProposalGenerator[State] with TransitionProbability[State]])(implicit
    rnd: Random): ProposalGenerator[State] with TransitionProbability[State] = {
    if (!probabilistic) GeneratorWrapperDeterministic(update, name)
    else {
      val mix = mixing.getOrElse {
        Generator().RandomShape()
      }
      val informedGenerator = GeneratorWrapperStochastic(update, cashedPosterior, name)
      val totalMix = mix.map(_._1).sum
      println(s"Mixing: ${totalMix}, other ${1.0 - totalMix}")
      MixtureProposal(0.2 *: mix + 0.8 *: informedGenerator)
    }
  }

  /** Runs the actual registration with the provided configuration through the passed parameters.
    *
    * @param initialState
    *   State from which the registration is started.
    * @param callBackLogger
    *   Logger to provide call back functionality to user after each iteration
    * @param acceptRejectLogger
    * @param evaluators
    *   Evaluator to be used if probabilistic registration is set
    * @param generators
    * @param probabilistic
    *   Flag which switches between probabilistic (true) and deterministic (false)
    * @param rnd
    *   Implicit random number generator.
    * @return
    *   Returns the best sample of the chain given the evaluator..
    */
  def run(
    initialState: State,
    callBackLogger: ChainStateLogger[State] = EmptyChainStateLogger(),
    acceptRejectLogger: AcceptRejectLogger[State] = EmptyAcceptRejectLogger(),
    evaluators: Option[Evaluator[State]] = None,
    generators: Option[ProposalGenerator[State] with TransitionProbability[State]] = None,
    probabilistic: Boolean = false
  )(implicit rnd: Random): State = {
    require(!probabilistic || probabilistic && evaluators.nonEmpty)
    val evaluator = evaluators.getOrElse(AcceptAll())
    val registrationEvaluator = EvaluatorWrapper(probabilistic, evaluator)
    val registrationGenerator = generatorCombined(probabilistic, generators)
    val bestSampleLogger = BestSampleLogger[State](registrationEvaluator)
    val logs = ChainStateLoggerContainer(Seq(callBackLogger, bestSampleLogger))
    val mhChain = MetropolisHastings[State](registrationGenerator, registrationEvaluator)

    val states = mhChain.iterator(initialState, acceptRejectLogger).loggedWith(logs)

    // we need to query if there is a next element, otherwise due to laziness the chain is not calculated
    states.take(initialState.general.maxIterations).dropWhile(state => !state.general.converged).hasNext

    val fit = bestSampleLogger.currentBestSample().get
    fit
  }
}
