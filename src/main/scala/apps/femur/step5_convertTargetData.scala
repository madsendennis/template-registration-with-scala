package apps.femur

import java.io.File

import api.registration.utils.modelViewer
import apps.util.{AlignmentTransforms, FileUtils}
import scalismo.geometry.Point3D
import scalismo.io.{LandmarkIO, MeshIO, StatisticalModelIO}
import scalismo.ui.api.ScalismoUI
import scalismo.ui.util.FileUtil
import scalismo.utils.Random.implicits.randomGenerator

object step5_convertTargetData extends App {
  scalismo.initialize()

  val targetFiles = FileUtils.getRecursiveListOfFiles(data.step3).filter(f => f.getName.endsWith(".stl")).sorted


  targetFiles.zipWithIndex.foreach { case(targetFile, i) =>
    val baseName = FileUtils.basename(targetFile)
    println(s"Processing: ${baseName}, index: ${i}")
    val target = MeshIO.readMesh(targetFile).get
    val targetLms = LandmarkIO.readLandmarksJson3D(new File(targetFile.getParent, baseName + ".json")).get

    val baseSplit = baseName.split('.')
    val outputName = s"VSD.case_${i+1}.${baseSplit(5)}"

    MeshIO.writeMesh(target, new File(data.step3, s"${outputName}.stl"))
    LandmarkIO.writeLandmarksJson(targetLms, new File(data.step3, s"${outputName}.json"))
  }
}