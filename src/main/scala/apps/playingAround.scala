package apps

import java.io.File

import api.{GeneralRegistrationState, RigidTransforms}
import api.registration.config.{CpdConfiguration, CpdRegistration, CpdRegistrationState, IcpConfiguration, IcpRegistration, IcpRegistrationState}
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.geometry._3D
import scalismo.io.{LandmarkIO, MeshIO, StatisticalModelIO}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.PointDistributionModel
import scalismo.ui.api.ScalismoUI

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

  // CPD
  val configCPD = CpdConfiguration(maxIterations = 20)
  val initState = CpdRegistrationState(GeneralRegistrationState(model, target), configCPD)
  val registratorCPD = new CpdRegistration()
  val finalCPD = registratorCPD.run(initState).general
  // ICP
  val configICP = IcpConfiguration(maxIterations = 20, initialSigma = 100, endSigma = 10)
  val icpState = IcpRegistrationState(finalCPD, configICP)
  val registratorICP = new IcpRegistration()
  val finalState = registratorICP.run(icpState).general

  val ui = ScalismoUI()
  val modelGroup = ui.createGroup("model")
  val initmodelGroup = ui.createGroup("initmodel")
  val targetGroup = ui.createGroup("target")
  val otherGroup = ui.createGroup("other")
  ui.show(initmodelGroup, model, "init")
  val showModel = ui.show(modelGroup, model, "model")
  ui.show(targetGroup, target, "target")
  ui.show(otherGroup, finalState.fit, "fit")
  showModel.shapeModelTransformationView.shapeTransformationView.coefficients = finalState.modelParameters
  showModel.shapeModelTransformationView.poseTransformationView.transformation = finalState.alignment
}
