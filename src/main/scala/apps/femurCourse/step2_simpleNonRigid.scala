package apps.femurCourse

import java.io.{File, IOException}

import api.registration.GpmmCpdRegistration
import api.registration.icp.NonRigidICPwithGPMMTriangle3D
import api.registration.utils.modelViewer
import apps.animations.ioStuff.{convertDiscreteFieldToVtkPolyData, writeVTKPdasVTK}
import apps.util.RegistrationComparison
import scalismo.common.{DiscreteField, DiscreteField3D, UnstructuredPointsDomain}
import scalismo.common.interpolation.{NearestNeighborInterpolator, TriangleMeshInterpolator3D}
import scalismo.geometry.{EuclideanVector, EuclideanVector3D, _3D}
import scalismo.io.{MeshIO, StatisticalModelIO}
import scalismo.kernels.{DiagonalKernel, GaussianKernel}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.{GaussianProcess, LowRankGaussianProcess, PointDistributionModel}
import scalismo.ui.api.{ScalismoUI, ScalismoUIHeadless}
import scalismo.ui.rendering.actor.{VectorFieldActor, VectorFieldActor3D}
import scalismo.utils.VtkHelpers
import vtk.{vtkCellArray, vtkDoubleArray, vtkFloatArray, vtkPoints, vtkPolyData, vtkPolyDataWriter, vtkVertex}

import scala.collection.immutable
import scala.util.{Failure, Success, Try}

object step2_simpleNonRigid extends App{
  scalismo.initialize()

  data.registeredMeshes.mkdirs()

  val targetIds = Seq(0).map(id => id.toString+".stl")

  val meshes = data.alignedMeshes.listFiles(_.getName.endsWith(".stl")).sorted.filter(f => targetIds.contains(f.getName))

//  val targetFile = meshes.find(f => f.getName.equals("41.stl")).get

  meshes.foreach{targetFile =>
    val gpmm = StatisticalModelIO.readStatisticalTriangleMeshModel3D(data.gpmm).get
    println(s"GPMM rank: ${gpmm.rank}, ref points: ${gpmm.reference.pointSet.numberOfPoints}")

    println(s"Processing: ${targetFile}")
    val target = MeshIO.readMesh(targetFile).get
    println(s"Target points: ${target.pointSet.numberOfPoints}")

//    val ui = ScalismoUIHeadless()
    val ui = ScalismoUI()
    val modelGroup = ui.createGroup("gpmm")
    val targetGroup = ui.createGroup("target")
    val gpmmView = ui.show(modelGroup, gpmm, "model")
    ui.show(targetGroup, target, targetFile.getName)

//    VtkHelpers.scalarArrayToVtkDataArray(df, new File("data/"))




    val mv = Option(modelViewer(gpmmView.shapeModelTransformationView, 10))

//    try {
//        val decGPMMcpd = gpmm.newReference(gpmm.reference.operations.decimate(500), NearestNeighborInterpolator())
//        val gpmmCPD = new GpmmCpdRegistration(decGPMMcpd, target.operations.decimate(2000), Seq(), Seq(), lambda = 1, w = 0, max_iterations = 40, modelView = mv)
//        val cpdFit = gpmmCPD.register(tolerance = 0.0001)
//
//        RegistrationComparison.evaluateReconstruction2GroundTruthDouble(targetFile.getName, gpmm.instance(cpdFit), target)

        val decGPMMicp = gpmm.newReference(gpmm.reference.operations.decimate(500), NearestNeighborInterpolator())
        val gpmmICP = new NonRigidICPwithGPMMTriangle3D(decGPMMicp, target.operations.decimate(2000), mv)
//        val icpFit = gpmmICP.Registration(max_iteration = 10, tolerance = 0.000000001, sigma2 = Seq(5.0, 2.0, 1.0, 0.1, 0.001, 0.00001, 0.0000000001), initialGPMM = cpdFit)
        val icpFit = gpmmICP.Registration(max_iteration = 10, tolerance = 0.000000001, sigma2 = Seq(10, 5.0, 1.0, 0.1)) //, initialGPMM = cpdFit)

//        val icpFitMesh = gpmm.instance(icpFit)
//
//        RegistrationComparison.evaluateReconstruction2GroundTruthDouble(targetFile.getName, icpFitMesh, target)

//        MeshIO.writeMesh(icpFitMesh, new File(data.registeredMeshes, targetFile.getName))
//    }
//    catch{
//        case _: Throwable => println(s"Error with ${targetFile}")
//    }
  }
}

// std icp ID: 0.stl average2surface: 0.30686905707228307 hausdorff: 2.9000570990369376
// norm icp ID:
//ID: 16.stl average2surface: 0.31717488278389255 hausdorff: 2.769287644764168