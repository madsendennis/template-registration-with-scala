package apps

import java.io.File

import api.registration.config.{CpdConfiguration, CpdRegistration, IcpConfiguration, IcpRegistration}
import api.registration.utils.AlignmentTransforms
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.geometry.{Point, _3D}
import scalismo.io.{LandmarkIO, MeshIO, StatisticalModelIO}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.PointDistributionModel
import scalismo.ui.api.ScalismoUI


object playingAround extends App {
  scalismo.initialize()

  val modelInit: PointDistributionModel[_3D, TriangleMesh] = StatisticalModelIO.readStatisticalTriangleMeshModel3D(new File("data/femur_gp_model_50-components.h5")).get

  val model = modelInit.newReference(modelInit.reference.operations.decimate(800), NearestNeighborInterpolator())
  val modelLM = LandmarkIO.readLandmarksJson3D(new File("data/femur_reference.json")).get
//  val target: TriangleMesh[_3D] = MeshIO.readMesh(new File("data/femur_target.stl")).get
  val target: TriangleMesh[_3D] = MeshIO.readMesh(new File("data/femur_target_aligned.stl")).get.operations.decimate(400)
  val targetLM = LandmarkIO.readLandmarksJson3D(new File("data/femur_target.json")).get

  println(s"Model points: ${model.reference.pointSet.numberOfPoints}")
  println(s"Target points: ${target.pointSet.numberOfPoints}")

//  // ICP
//  val config = IcpConfiguration(maxIterations = 10, initialSigma = 100, endSigma = 1)
//  val registrator = new IcpRegistration(target, config, model)
//  val finalState = registrator.run(target, Option(targetLM), model, Option(modelLM))

  // CPD
  val config = CpdConfiguration(maxIterations = 50)
  val registrator = new CpdRegistration(target, config, model)
  val finalState = registrator.run(target, Option(targetLM), model, Option(modelLM))


  val fit = finalState.fit
  val m = finalState.model

  val ui = ScalismoUI()
  val modelGroup = ui.createGroup("model")
  val targetGroup = ui.createGroup("target")
  val otherGroup = ui.createGroup("other")
  ui.show(modelGroup, m, "model")
  ui.show(targetGroup, target, "target")
  ui.show(otherGroup, fit, "fit")
}
