//package apps.paperFigures
//
//import java.io.File
//
//import api.RigidTransforms
//import api.gpmm.{GPMMTriangleMesh3D, PointSetHelper}
//import api.registration.utils.GPMMHelper
//import apps.DemoDatasetLoader
//import apps.registration.{DemoCPD, DemoICP}
//import scalismo.common.interpolation.TriangleMeshInterpolator3D
//import scalismo.geometry._3D
//import scalismo.io.{MeshIO, StatisticalModelIO}
//import scalismo.mesh.TriangleMesh
//import scalismo.statisticalmodel.PointDistributionModel
//import scalismo.ui.api.ScalismoUI
//import scalismo.utils.Random.implicits.randomGenerator
//
//object BunnyDeformations extends App {
//  scalismo.initialize()
//
//  val modelPath = new File("data/bunny/bunny_gauss.h5")
//
//  def createModel(ref: TriangleMesh[_3D]): PointDistributionModel[_3D, TriangleMesh] = {
//    val lowRes = ref.operations.decimate(10000)
//
//    val ps = PointSetHelper[_3D, TriangleMesh](lowRes)
//    val maxDist = ps.maximumPointDistance(lowRes.pointSet)
//    val minDist = ps.minimumPointDistance(lowRes.pointSet)
//
//    println(s"Max: ${maxDist}, Min: ${minDist}")
//
//    val model = GPMMTriangleMesh3D(lowRes, relativeTolerance = 0.1).Gaussian(maxDist / 4, maxDist / 8)
//
//    val highResModel = model.newReference(ref, TriangleMeshInterpolator3D())
//    StatisticalModelIO.writeStatisticalTriangleMeshModel3D(highResModel, modelPath)
//    model
//  }
//
//  val model = StatisticalModelIO.readStatisticalTriangleMeshModel3D(modelPath).getOrElse {
//    println("Model not found, creating ...")
//    val (ref, _) = DemoDatasetLoader.referenceBunny()
//    createModel(ref)
//  }
//  val (target, _) = DemoDatasetLoader.targetBunny()
//
//  //  MeshIO.writeScalarMeshField[Double](color, new File(s"data/armadillo/color/${kernelSelect.name}_${dataType}.vtk"))
//
//  DemoICP.run(model, target, discretization = 1000, maxIterations = 1000, probabilistic = false)
//
//  val ui = ScalismoUI()
//  val modelGroup = ui.createGroup("model")
//  val targetGroup = ui.createGroup("target")
//
//  //    ui.show(ref, "s")
//  //  ui.show(lms, "lms")
//  ui.show(modelGroup, model, "model")
//  ui.show(targetGroup, target, "target")
//
//}
