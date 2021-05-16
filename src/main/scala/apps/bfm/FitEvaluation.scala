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
import apps.util.{LogHelper, PosteriorVariability, RegistrationComparison}
import scalismo.io.MeshIO
import scalismo.ui.api.ScalismoUI

object FitEvaluation {

  def main(args: Array[String]): Unit = {
    scalismo.initialize()

    val outputFolder = new File(generalPath, "output/IPE0.1/")

    val targetFiles = outputFolder.listFiles(f => f.getName.contains("_target.ply"))

    val dists = targetFiles.map{f =>
      val target = MeshIO.readMesh(f).get
      val complete = MeshIO.readMesh(new File(f.toString.replace("target", "complete"))).get
      RegistrationComparison.evaluateReconstruction2GroundTruthBoundaryAware(f.getName, target, complete)
    }
    val avg = dists.map(_._1).sum / dists.length.toDouble
    val haus = dists.map(_._2).sum / dists.length.toDouble
    println(s"avg: ${avg}, haus: ${haus}")
  }
}
