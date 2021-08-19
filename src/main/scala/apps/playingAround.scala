package apps

import java.io.File

import api.registration.config.{CPD, ICP}
import api.{DefaultRegistrationPars, GiNGR, GiNGRConfig}
import scalismo.geometry._3D
import scalismo.io.{LandmarkIO, MeshIO, StatisticalModelIO}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.{MultivariateNormalDistribution, PointDistributionModel}
import scalismo.ui.api.ScalismoUI


object playingAround extends App {
  scalismo.initialize()

  val model: PointDistributionModel[_3D, TriangleMesh] = StatisticalModelIO.readStatisticalTriangleMeshModel3D(new File("data/femur_gp_model_50-components.h5")).get
  val modelLM = LandmarkIO.readLandmarksJson3D(new File("data/femur_reference.json")).get
  val target: TriangleMesh[_3D] = MeshIO.readMesh(new File("data/femur_target.stl")).get
  val targetLM = LandmarkIO.readLandmarksJson3D(new File("data/femur_target.json")).get

  // ICP Setting
  //  val regConf: GiNGRConfig = new ICP
  //  val gingr = new GiNGR(model = model, modelLandmarks = Option(modelLM), registrationConfig = regConf)
  //  val regConfig = DefaultRegistrationPars(max_iterations = 50, tolerance = 0.0001, landmarkAlignTarget = true)

  // CPD Setting
  val regConf: GiNGRConfig = new CPD
  val gingr = new GiNGR(model = model, modelLandmarks = Option(modelLM), registrationConfig = regConf)
  val regConfig = DefaultRegistrationPars(max_iterations = 20, tolerance = 0.0001, landmarkAlignTarget = true)

  val (m, a, t) = gingr.Registration(target, targetLandmarks = Option(targetLM), regConfig = regConfig)
  val trans = t.rigidTransform
  val fit = m.instance(a)


  val ui = ScalismoUI()
  val modelGroup = ui.createGroup("model")
  val targetGroup = ui.createGroup("target")
  val otherGroup = ui.createGroup("other")
  ui.show(modelGroup, m, "model")
  ui.show(targetGroup, target, "target")
  ui.show(otherGroup, fit, "fit")
}
