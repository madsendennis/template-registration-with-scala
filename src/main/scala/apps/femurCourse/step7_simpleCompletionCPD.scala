package apps.femurCourse

import java.awt.Color
import java.io.File

import api.registration.{GpmmBcpdRegistration, GpmmCpdRegistration}
import api.registration.utils.{RigidTransforms, modelViewer}
import apps.util.{AlignmentTransforms, FileUtils, RegistrationComparison}
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.geometry.{Point3D, _3D}
import scalismo.io.{LandmarkIO, MeshIO, StatisticalModelIO}
import scalismo.mesh.TriangleMesh
import scalismo.ui.api.{ScalismoUI, ScalismoUIHeadless}
import scalismo.utils.Random.implicits.randomGenerator

object step7_simpleCompletionCPD extends App {
  scalismo.initialize()

  val completedPath = new File(data.completed, "bcpd_pca")
  completedPath.mkdirs()

  val modelInit = StatisticalModelIO.readStatisticalTriangleMeshModel3D(data.pca).get
  val modelLmsInit = LandmarkIO.readLandmarksJson3D(data.referenceLms).get

  val targetFiles = data.step3.listFiles(f => f.getName.endsWith(".stl")).sorted

//    val targetFile = targetFiles(1) //.find(f => f.getName.equals("VSD.case_2.101148.stl")).get
  targetFiles.foreach { targetFile =>

    println(s"Processing: ${targetFile}")
    val baseName = FileUtils.basename(targetFile)
    val target = MeshIO.readMesh(targetFile).get
    val targetLms = LandmarkIO.readLandmarksJson3D(new File(targetFile.getParent, baseName + ".json")).get

    println(s"Target points: ${target.pointSet.numberOfPoints}")
    val transform = AlignmentTransforms.computeTransform(modelLmsInit, targetLms, Point3D(0, 0, 0))
    val gpmm = modelInit.transform(transform)
    val modelLms = modelLmsInit.map(_.transform(transform))

    val commonLmNamesInit = modelLms.map(_.id) intersect targetLms.map(_.id)
    val commonLmNames = commonLmNamesInit.filter(f => !f.toLowerCase.contains("shaft"))
    val trainingData = commonLmNames.map(name => (gpmm.mean.pointSet.findClosestPoint(modelLms.find(_.id == name).get.point).id, targetLms.find(_.id == name).get.point)).toIndexedSeq
    val gpmmPos = gpmm.posterior(trainingData, 100.0)

//    val ui: ScalismoUIHeadless = ScalismoUIHeadless()
    val ui: ScalismoUI = ScalismoUI()
    val modelGroup = ui.createGroup("gpmm")
    val targetGroup = ui.createGroup("target")
    val gpmmView = ui.show(modelGroup, gpmmPos, "model")
    val showTarget = ui.show(targetGroup, target, targetFile.getName)
    showTarget.color = Color.YELLOW
//    ui.show(targetGroup, targetLms, "lms")
//    ui.show(modelGroup, modelLms, "lms")

    val mv = if (ui.isInstanceOf[ScalismoUI]) Option(modelViewer(gpmmView.shapeModelTransformationView, 10)) else None

    val decGPMMcpd = gpmmPos.newReference(gpmmPos.reference.operations.decimate(200), NearestNeighborInterpolator())
//    val gpmmCPD = new GpmmCpdRegistration[_3D, TriangleMesh](decGPMMcpd, target.operations.decimate(10000), Seq(), Seq(), lambda = 1, w = 0, max_iterations = 100, modelView = mv)
//    val cpdFit = gpmmCPD.register(tolerance = 0.000001)
//    val cpdFitMesh = gpmm.instance(cpdFit)

    val gpmmCPD = new GpmmBcpdRegistration[_3D, TriangleMesh](decGPMMcpd, target.operations.decimate(400), lambda = 1, w = 0, max_iterations = 100, modelView = mv)
    val cpdFit = gpmmCPD.register(tolerance = 0.000001, transformationType = RigidTransforms)
    val cpdFitMesh = gpmmPos.instance(cpdFit._1).transform(cpdFit._2.rigidTransform)

    RegistrationComparison.evaluateReconstruction2GroundTruth(baseName, target, cpdFitMesh)

    MeshIO.writeMesh(cpdFitMesh, new File(completedPath, targetFile.getName))
  }
}
