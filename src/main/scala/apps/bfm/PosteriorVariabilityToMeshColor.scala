/*
 *  Copyright University of Basel, Graphics and Vision Research Group
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

package apps.bfm

import java.io.File

import api.sampling.ModelFittingParameters
import api.sampling.loggers.{JSONAcceptRejectLogger, jsonLogFormat}
import apps.bfm.Paths.generalPath
import apps.util.{LogHelper, PosteriorVariability}
import scalismo.io.MeshIO
import scalismo.ui.api.ScalismoUI

object PosteriorVariabilityToMeshColor {

  def main(args: Array[String]): Unit = {
    scalismo.initialize()

    val outputFolder = new File(generalPath, "output/IPE0.1/")
      outputFolder.mkdirs()

//    (0 until 10).foreach { id =>
      val id = 2
      val (model, targetName, gt, target, targetLogFile) = LoadTestData.modelAndTarget(id, 100)

      val logObj = new JSONAcceptRejectLogger[ModelFittingParameters](targetLogFile)
      val logInit: IndexedSeq[jsonLogFormat] = logObj.loadLog()
      val burnInPhase = 500

      val logSamples = LogHelper.samplesFromLog(logInit, takeEveryN = 100, total = 10000, burnInPhase)
      println(s"Number of samples from log: ${logSamples.length}/${logInit.length - burnInPhase}")
      val logShapes = scala.util.Random.shuffle(LogHelper.logSamples2shapes(model, logSamples.map(_._1))).take(500)

      val best = ModelFittingParameters.transformedMesh(model, logObj.getBestFittingParsFromJSON)

//      val colorMap_normalVariance = PosteriorVariability.computeDistanceMapFromMeshesNormal(logShapes, best, sumNormals = true)
//      val colorMap_posteriorEstimate = PosteriorVariability.computeDistanceMapFromMeshesTotal(logShapes, best)

      val ui = ScalismoUI(s"Posterior visualization - $targetName")
      val modelGroup = ui.createGroup("model")
      val colorGroup = ui.createGroup("color")
      val targetGroup = ui.createGroup("target")
      val showModel = ui.show(modelGroup, model, "model")
      showModel.meshView.opacity = 0.0
//      ui.show(colorGroup, colorMap_posteriorEstimate, "posterior").opacity = 0.0
//      ui.show(colorGroup, colorMap_normalVariance, "normal").opacity = 0.0
      ui.show(colorGroup, best, "best-fit")
      ui.show(targetGroup, target, s"${targetName}")
      ui.show(targetGroup, gt, "gt")
          val rndGroup = ui.createGroup("random")
          scala.util.Random.shuffle(logShapes).take(10).zipWithIndex.foreach(m => ui.show(rndGroup, m._1, m._2.toString))
//      MeshIO.writeMesh(target, new File(outputFolder, s"${id}_target.ply"))
//      MeshIO.writeMesh(best, new File(outputFolder, s"${id}_complete.ply"))
//      MeshIO.writeScalarMeshField[Double](colorMap_posteriorEstimate, new File(outputFolder, s"${id}_pos.vtk"))
//      MeshIO.writeScalarMeshField[Double](colorMap_normalVariance, new File(outputFolder, s"${id}_norm.vtk"))

//    }
  }
}
