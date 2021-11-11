package apps

import api.helper.{CallBackFunctions, RegistrationComparison}
import api.{GeneralRegistrationState, GingrAlgorithm, GingrConfig, GingrRegistrationState, GlobalTranformationType, ModelFittingParameters, ProbabilisticSettings, RigidTransforms}
import api.registration.config.StateHandler
import api.sampling.IndependtPoints
import api.sampling.evaluators.{EvaluationMode, ModelToTargetEvaluation}
import api.sampling.loggers.JSONStateLogger
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.geometry._3D
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.PointDistributionModel
import scalismo.ui.api.{PointDistributionModelViewControlsTriangleMesh3D, ScalismoUI, ScalismoUIHeadless, SimpleAPI, SimpleAPIDefaultImpl, TriangleMeshView}
import scalismo.utils.Random

class SimpleRegistrator[State <: GingrRegistrationState[State], Algorithm <: GingrAlgorithm[State], Config <: GingrConfig](
  model: PointDistributionModel[_3D, TriangleMesh],
  target: TriangleMesh[_3D],
  algorithm: Algorithm,
  config: Config,
  transform: GlobalTranformationType = RigidTransforms,
  evaluatorUncertainty: Double = 1.0,
  evaluatedPoints: Option[Int] = None,
  evaluationMode: EvaluationMode = ModelToTargetEvaluation,
  showInUI: Boolean = true
)(implicit stateHandler: StateHandler[State, Config], rnd: Random) {

  def createInitialState(model: PointDistributionModel[_3D, TriangleMesh], target: TriangleMesh[_3D]): State = {
    val generalState = GeneralRegistrationState(model, target, transform)
    stateHandler.initialize(generalState, config)
  }

  lazy val initialState: State = createInitialState(model, target)

  val ui: SimpleAPI with SimpleAPIDefaultImpl = if (showInUI) ScalismoUI() else ScalismoUIHeadless()
  private val modelGroup = ui.createGroup("model")
  private val targetGroup = ui.createGroup("target")
  private val finalGroup = ui.createGroup("final")

  val modelView: PointDistributionModelViewControlsTriangleMesh3D = ui.show(modelGroup, model, "model")
  val targetView: TriangleMeshView = ui.show(targetGroup, target, "target")

  private def decimateState(state: Option[State], modelPoints: Int, targetPoints: Int): State = {
    val newRef = model.reference.operations.decimate(modelPoints)
    val decimatedModel = model.newReference(newRef, NearestNeighborInterpolator())
    val decimatedTarget = target.operations.decimate(targetPoints)
    val initState = state.getOrElse(createInitialState(decimatedModel, decimatedTarget))
    initState.updateGeneral(
      initState.general.copy(
        model = decimatedModel,
        target = decimatedTarget,
        fit = ModelFittingParameters.modelInstanceShapePoseScale(decimatedModel, initState.general.modelParameters)
      ))
  }

  def run(state: State = initialState, probabilistic: Boolean = false): State = {
    println(s"RUN!!! model: ${state.general.model.reference.pointSet.numberOfPoints}, target: ${state.general.target.pointSet.numberOfPoints}")
    modelView.shapeModelTransformationView.shapeTransformationView.coefficients = state.general.modelParameters.shape.parameters
    modelView.shapeModelTransformationView.poseTransformationView.transformation = state.general.modelParameters.rigidTransform
    val evaluator: IndependtPoints[State] = IndependtPoints(
      state = state,
      uncertainty = evaluatorUncertainty,
      mode = evaluationMode,
      evaluatedPoints = evaluatedPoints
    )
    val jsonLogger = if (probabilistic) Some(JSONStateLogger(evaluator)) else None
    val visualLogger = CallBackFunctions.visualLogger(jsonLogger, modelView)

    val finalState = algorithm.run(
      initialState = state,
      acceptRejectLogger = jsonLogger,
      callBackLogger = visualLogger,
      probabilisticSettings = if (probabilistic) Some(ProbabilisticSettings[State](evaluator)) else None
    )
    val fit = ModelFittingParameters.modelInstanceShapePoseScale(model, finalState.general.modelParameters)

    ui.show(finalGroup, fit, "fit")
    RegistrationComparison.evaluateReconstruction2GroundTruthBoundaryAware("", fit, target)
    finalState
  }

  def runDecimated(modelPoints: Int, targetPoints: Int, state: Option[State] = None, probabilistic: Boolean = false): State = {
    val initState = decimateState(state, modelPoints, targetPoints)
    run(initState, probabilistic)
  }

}
