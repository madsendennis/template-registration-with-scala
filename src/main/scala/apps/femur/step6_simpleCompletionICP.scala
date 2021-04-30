package apps.femur

import java.awt.Color
import java.io.File

import api.registration.icp.{NonRigidICPwithGPMMTriangle3D, RigidICPtransform}
import api.registration.utils.{TargetToReference, modelViewer}
import apps.util.{AlignmentTransforms, FileUtils, RegistrationComparison}
import scalismo.geometry.Point3D
import scalismo.io.{LandmarkIO, MeshIO, StatisticalModelIO}
import scalismo.ui.api.{ScalismoUI, ScalismoUIHeadless}
import scalismo.utils.Random.implicits.randomGenerator

object step6_simpleCompletionICP extends App {
  scalismo.initialize()

  val completedPath = new File(data.completed, "icpRealign")
  completedPath.mkdirs()

  val modelInit = StatisticalModelIO.readStatisticalTriangleMeshModel3D(data.augmented).get
  val modelLmsInit = LandmarkIO.readLandmarksJson3D(data.referenceLms).get

  val targetFiles = data.step3.listFiles(f => f.getName.endsWith(".stl")).sorted

  val targetFile = targetFiles.find(f => f.getName.equals("VSD.case_2.101148.stl")).get
  //  targetFiles.par.foreach { targetFile =>

  println(s"Processing: ${targetFile}")
  val baseName = FileUtils.basename(targetFile)
  val target = MeshIO.readMesh(targetFile).get
  val targetLms = LandmarkIO.readLandmarksJson3D(new File(targetFile.getParent, baseName + ".json")).get

  println(s"Target points: ${target.pointSet.numberOfPoints}")
  val transform = AlignmentTransforms.computeTransform(modelLmsInit, targetLms, Point3D(0, 0, 0))
  val gpmm0 = modelInit.transform(transform)
  val modelLms0 = modelLmsInit.map(_.transform(transform))

  val icp = new RigidICPtransform(gpmm0.mean, target)
  val icpTrans = icp.Registration(max_iteration = 10, tolerance = 0.0001, direction = TargetToReference)

  val gpmm = gpmm0.transform(icpTrans)
  val modelLms = modelLms0.map(_.transform(icpTrans))

  val commonLmNamesInit = modelLms.map(_.id) intersect targetLms.map(_.id)
  val commonLmNames = commonLmNamesInit.filter(f => !f.toLowerCase.contains("shaft"))
  val trainingData = commonLmNames.map(name => (gpmm.mean.pointSet.findClosestPoint(modelLms.find(_.id == name).get.point).id, targetLms.find(_.id == name).get.point)).toIndexedSeq
  val gpmmPos = gpmm.posterior(trainingData, 100.0)

  val ui: ScalismoUIHeadless = ScalismoUIHeadless()
//  val ui: ScalismoUI = ScalismoUI()
  val modelGroup = ui.createGroup("gpmm")
  val targetGroup = ui.createGroup("target")
  val gpmmView = ui.show(modelGroup, gpmmPos, "model")
  val showTarget = ui.show(targetGroup, target, targetFile.getName)
  showTarget.color = Color.YELLOW
  ui.show(targetGroup, targetLms, "lms")
  ui.show(modelGroup, modelLms, "lms")


  val mv = if (ui.isInstanceOf[ScalismoUI]) Option(modelViewer(gpmmView.shapeModelTransformationView, 10)) else None

  val decGPMMicp = gpmmPos //gpmmPos.newReference(gpmmPos.reference.operations.decimate(10000), NearestNeighborInterpolator())
  val gpmmICP = new NonRigidICPwithGPMMTriangle3D(decGPMMicp, target.operations.decimate(2000), mv)
  val icpFit = gpmmICP.Registration(max_iteration = 20, tolerance = 0.000000001, sigma2 = Seq(100, 10, 5, 2.0, 1.0), direction = TargetToReference)
  val icpFitMesh = gpmmPos.instance(icpFit)

  RegistrationComparison.evaluateReconstruction2GroundTruth(baseName, target, icpFitMesh)

  MeshIO.writeMesh(icpFitMesh, new File(completedPath, targetFile.getName))
  //  }
}
