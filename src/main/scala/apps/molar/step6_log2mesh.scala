package apps.molar

import java.awt.Color
import java.io.File

import api.sampling.ModelFittingParameters
import api.sampling.loggers.JSONAcceptRejectLogger
import apps.util.{FileUtils, RegistrationComparison}
import scalismo.io.{MeshIO, StatisticalModelIO}
import scalismo.ui.api.ScalismoUI

object step6_log2mesh extends App {
  scalismo.initialize()

  val logPath = new File(data.log, "step5")
  val logFiles = logPath.listFiles(_.getName.endsWith(".json")).filter(!_.getName.contains("_init.json")).sorted
  println(s"LogPath: ${logPath}")
  val targetPath = data.rawCrowns

  logFiles.map{ log =>
    val baseName = FileUtils.basename(log)

    val model = StatisticalModelIO.readStatisticalMeshModel(data.augmentedAligned).get.truncate(400)

    val logObj = new JSONAcceptRejectLogger[ModelFittingParameters](log)
    val bestRegistration = ModelFittingParameters.transformedMesh(model, logObj.getBestFittingParsFromJSON)

    val target = MeshIO.readMesh(new File(targetPath, baseName+".stl")).get

    RegistrationComparison.evaluateReconstruction2GroundTruth(baseName, target, bestRegistration)

//    val ui = ScalismoUI()
//    ui.show(target, "target").color = Color.YELLOW
//    ui.show(bestRegistration, "MAP")
//    scala.io.StdIn.readLine("paused. Enter to continue...")
//    ui.close()
    MeshIO.writeMesh(bestRegistration, new File(data.registeredMeshesCrown, baseName+".ply"))
  }
  println("All done")
}
