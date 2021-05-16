package apps.femurCourse

import java.awt.Color
import java.io.File

import api.sampling.ModelFittingParameters
import api.sampling.loggers.{JSONAcceptRejectLogger, jsonLogFormat}
import apps.util.{FileUtils, LogHelper, PosteriorVariability, RegistrationComparison}
import scalismo.io.{MeshIO, StatisticalModelIO}
import scalismo.ui.api.ScalismoUI

object step10_visualizePreg extends App {
  scalismo.initialize()

  val logPath = new File(data.data, "finalLogs")
  val completedPath = new File(data.completed, "myBest")
  val targetPath = data.step3.listFiles(_.getName.endsWith(".stl"))

  val bestColorPath = new File(completedPath, "color")
  bestColorPath.mkdirs()


  val logFiles = logPath.listFiles(f => f.getName.endsWith(".json")).sorted

  //  val logFile = logFiles(0) //.find(f => f.getName.equals("VSD.case_2.101148.stl")).get
  logFiles.foreach { logFile =>

    println(s"Processing: ${logFile}")
    val baseName = FileUtils.basename(logFile)

    val target = MeshIO.readMesh(targetPath.find(_.getName.endsWith(s"${baseName}.stl")).get).get
    val best = MeshIO.readMesh(completedPath.listFiles().find(_.getName.endsWith(s"${baseName}.stl")).get).get
    val model = StatisticalModelIO.readStatisticalMeshModel(logPath.listFiles().find(_.getName.endsWith(s"${baseName}.h5")).get).get
    val logObj = new JSONAcceptRejectLogger[ModelFittingParameters](logFile)
    val logInit: IndexedSeq[jsonLogFormat] = logObj.loadLog()
    println(s"Log length: ${logInit.length}")
    val burnInPhase = 200

    val logSamples = LogHelper.samplesFromLog(logInit, takeEveryN = 50, total = 10000, burnInPhase)
    println(s"Number of samples from log: ${logSamples.length}/${logInit.length - burnInPhase}")
    val logShapes = LogHelper.logSamples2shapes(model, logSamples.map(_._1))

    val colorMap_normalVariance = PosteriorVariability.computeDistanceMapFromMeshesNormal(logShapes, best, sumNormals = true)
    val colorMap_posteriorEstimate = PosteriorVariability.computeDistanceMapFromMeshesTotal(logShapes, best)

    val ui: ScalismoUI = ScalismoUI()
    val targetGroup = ui.createGroup("target")
    val completeGroup = ui.createGroup("complete")

    val showTarget = ui.show(targetGroup, target, baseName)
    showTarget.color = Color.RED
    val showComplete = ui.show(completeGroup, best, "best")
    ui.show(completeGroup, colorMap_posteriorEstimate, "posterior")
    ui.show(completeGroup, colorMap_normalVariance, "normal")

    MeshIO.writeScalarMeshField(colorMap_posteriorEstimate, new File(bestColorPath, s"${baseName}_pos.vtk"))
    MeshIO.writeScalarMeshField(colorMap_normalVariance, new File(bestColorPath, s"${baseName}_normal.vtk"))

    RegistrationComparison.evaluateReconstruction2GroundTruth(baseName, target, best)
  }
}
