//package apps.paperFigures
//
//import java.io.File
//
//import api.gpmm.{GPMMTriangleMesh3D, PointSetHelper}
//import apps.DemoDatasetLoader
//import scalismo.common.interpolation.TriangleMeshInterpolator3D
//import scalismo.geometry._3D
//import scalismo.io.{MeshIO, StatisticalModelIO}
//import scalismo.mesh.TriangleMesh
//import scalismo.ui.api.ScalismoUI
//import scalismo.utils.Random.implicits.randomGenerator
//
//object KernelsArmadillos extends App {
//  scalismo.initialize()
//
//  val (ref, lms) = DemoDatasetLoader.targetArmadillo()
////  val (ref, lms) = DemoDatasetLoader.referenceArmadillo()
//
//  val dataType = "karate_new"
//
//  val lowRes = ref.operations.decimate(1000)
//
//  val toeLm = lms.find(f => f.id == "J").get
//  val toeId = ref.pointSet.findClosestPoint(toeLm.point).id
//
//  val ps = PointSetHelper[_3D, TriangleMesh](lowRes)
//  val maxDist = ps.maximumPointDistance(lowRes.pointSet)
//  val minDist = ps.minimumPointDistance(lowRes.pointSet)
//
//  val allKernels: Seq[WhichKernel] = Seq(InvLapDot) //Seq(InvLap, InvLapDot, Gauss, GaussMirror)
//
////  val kernelSelect: WhichKernel = InvLapDot
//
//  allKernels.map { kernelSelect =>
//    val model = kernelSelect match {
//      case InvLap => GPMMTriangleMesh3D(lowRes, relativeTolerance = 0.01).InverseLaplacian(50)
//      case InvLapDot => GPMMTriangleMesh3D(lowRes, relativeTolerance = 0.01).InverseLaplacianDot(0.005, 1.0)
//      case Gauss => GPMMTriangleMesh3D(lowRes, relativeTolerance = 0.01).Gaussian(maxDist / 4, maxDist / 8)
//      case GaussMirror => GPMMTriangleMesh3D(lowRes, relativeTolerance = 0.01).GaussianSymmetry(maxDist / 4, maxDist / 8)
//    }
//
//    StatisticalModelIO.writeStatisticalTriangleMeshModel3D(model, new File(s"data/armadillo/models/${kernelSelect.name}_${dataType}.h5"))
//
//    val highModel = model.newReference(ref, TriangleMeshInterpolator3D())
//
//    val gpmmHelp = GPMMTriangleMesh3D(highModel.reference, relativeTolerance = 0.0)
//    val color = gpmmHelp.computeDistanceAbsMesh(highModel, toeId)
//
//    MeshIO.writeScalarMeshField[Double](color, new File(s"data/armadillo/color/${kernelSelect.name}_${dataType}.vtk"))
//
//    //  val model = GPMMTriangleMesh3D(lowRes, relativeTolerance = 0.01).InverseLaplacianDot(0.05, 1.0)
//
//    //  val gpmmGauss = gpmmHelp.AutomaticGaussian().truncate(truncate)
//    //  val gpmmInvLap = gpmmHelp.InverseLaplacian(50).truncate(truncate)
//    //  val gpmmInvLapDot = gpmmHelp.InverseLaplacianDot(0.05, 1.0).truncate(truncate)
//
//    val ui = ScalismoUI(kernelSelect.name)
////    ui.show(ref, "s")
//    //  ui.show(lms, "lms")
//    ui.show(color, "color")
//
//  }
//}
