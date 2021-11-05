package apps

import java.io.File

import api.{GeneralRegistrationState, NoTransforms, RigidTransforms}
import api.registration.config.{CpdConfiguration, CpdRegistration, CpdRegistrationState, IcpConfiguration, IcpRegistration, IcpRegistrationState}
import api.sampling.IndependtPoints
import api.sampling.evaluators.{IndependentPointDistanceEvaluator, ModelToTargetEvaluation}
import api.sampling.loggers.JSONStateLogger
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.geometry._3D
import scalismo.io.{LandmarkIO, MeshIO, StatisticalModelIO}
import scalismo.mesh.TriangleMesh
import scalismo.sampling.DistributionEvaluator
import scalismo.sampling.loggers.ChainStateLogger
import scalismo.statisticalmodel.PointDistributionModel
import scalismo.ui.api.ScalismoUI
import scalismo.utils.Random.implicits.randomGenerator

object playingAround extends App {
  scalismo.initialize()

  val modelInit: PointDistributionModel[_3D, TriangleMesh] = StatisticalModelIO.readStatisticalTriangleMeshModel3D(new File("data/femur_gp_model_50-components.h5")).get

  val model = modelInit.newReference(modelInit.reference.operations.decimate(800), NearestNeighborInterpolator())
  val modelLM = LandmarkIO.readLandmarksJson3D(new File("data/femur_reference.json")).get
  val target: TriangleMesh[_3D] = MeshIO.readMesh(new File("data/femur_target.stl")).get.operations.decimate(400)
//  val target: TriangleMesh[_3D] = MeshIO.readMesh(new File("data/femur_target_aligned.stl")).get.operations.decimate(400)
  val targetLM = LandmarkIO.readLandmarksJson3D(new File("data/femur_target.json")).get

  println(s"Model points: ${model.reference.pointSet.numberOfPoints}")
  println(s"Target points: ${target.pointSet.numberOfPoints}")

  val ui = ScalismoUI()
  val modelGroup = ui.createGroup("model")
  val initmodelGroup = ui.createGroup("initmodel")
  val targetGroup = ui.createGroup("target")
  val otherGroup = ui.createGroup("other")
  ui.show(initmodelGroup, model, "init")
  val modelView = ui.show(modelGroup, model, "model")
  ui.show(targetGroup, target, "target")

  // CPD
  val configCPD = CpdConfiguration(maxIterations = 100)
  val initState: CpdRegistrationState = CpdRegistrationState(GeneralRegistrationState(model, target, transform = RigidTransforms), configCPD)
  val registratorCPD = new CpdRegistration()
  val t1 = System.nanoTime

  val myEval = IndependtPoints(initState, 1.0)

  val jsonLogger = JSONStateLogger(myEval)

  case class visualLogger() extends ChainStateLogger[CpdRegistrationState] {
    var counter = 0
    override def logState(sample: CpdRegistrationState): Unit = {
      counter += 1
      if (counter % 10 == 0) {
        println(s"Iteration: ${counter}/${sample.general.maxIterations}")
        jsonLogger.printAcceptInfo()
        modelView.shapeModelTransformationView.poseTransformationView.transformation = sample.general.rigidTransform
        modelView.shapeModelTransformationView.shapeTransformationView.coefficients = sample.general.modelParameters.shape.parameters
      }
    }
  }

  val finalCPD = registratorCPD.run(initState, callBackLogger = visualLogger(), acceptRejectLogger = jsonLogger, evaluators = Some(myEval), probabilistic = true)
  val duration = (System.nanoTime - t1) / 1e9d
  println(s"Registration time: ${duration}")

  // ICP
//  val configICP = IcpConfiguration(maxIterations = 20, initialSigma = 100, endSigma = 10)
//  val icpState = IcpRegistrationState(finalCPD, configICP)
//  val registratorICP = new IcpRegistration()
//  val finalState = registratorICP.run(icpState).general

//  ui.show(otherGroup, finalState.fit, "fit")
//  showModel.shapeModelTransformationView.shapeTransformationView.coefficients = finalState.modelParameters
//  showModel.shapeModelTransformationView.poseTransformationView.transformation = finalState.alignment
}
