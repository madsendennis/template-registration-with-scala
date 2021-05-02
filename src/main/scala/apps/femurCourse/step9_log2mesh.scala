package apps.femurCourse

import java.awt.Color
import java.io.File

import apps.util.RegistrationComparison
import api.sampling.ModelFittingParameters
import api.sampling.loggers.{JSONAcceptRejectLogger, jsonLogFormat}
import apps.util.FileUtils
import scalismo.io.{MeshIO, StatisticalModelIO}
import scalismo.ui.api.ScalismoUI

object step9_log2mesh extends App {
  scalismo.initialize()

  val logPath = new File(data.data, "log2_2mm")
  val logFiles = logPath.listFiles(_.getName.endsWith(".json")).filter(!_.getName.contains("_")).sorted
  println(s"LogPath: ${logPath}")
  val completedPath = new File(data.completed, "preg_pca2_2mm")
  val targetPath = data.step3
  completedPath.mkdirs()

  logFiles.map{ log =>
    val baseName = FileUtils.basename(log)

    val model = StatisticalModelIO.readStatisticalMeshModel(new File(log.getParent, baseName+".h5")).get

    val logObj = new JSONAcceptRejectLogger[ModelFittingParameters](log)
    val bestRegistration = ModelFittingParameters.transformedMesh(model, logObj.getBestFittingParsFromJSON)

    val target = MeshIO.readMesh(new File(targetPath, baseName+".stl")).get

    RegistrationComparison.evaluateReconstruction2GroundTruth(baseName, target, bestRegistration)

//    val ui = ScalismoUI()
//    ui.show(target, "target").color = Color.YELLOW
//    ui.show(bestRegistration, "MAP")
//    scala.io.StdIn.readLine("paused. Enter to continue...")
//    ui.close()
    MeshIO.writeMesh(bestRegistration, new File(completedPath, baseName+".stl"))
  }
  println("All done")
}
