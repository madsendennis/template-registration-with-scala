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

package apps.molar

import java.io.File

import scalismo.io.{MeshIO, StatisticalModelIO}
import scalismo.statisticalmodel.StatisticalMeshModel
import scalismo.statisticalmodel.dataset.DataCollection
import scalismo.ui.api.ScalismoUI
import scalismo.utils.Random

object step3_computePCAmodel {
  implicit val random: Random = Random(1024)

  def main(args: Array[String]) {
    scalismo.initialize()

    val alignedMeshesPath = data.registeredMeshesCoarse.listFiles(_.getName.endsWith(".ply"))
    val meshes = alignedMeshesPath.map(f => MeshIO.readMesh(f).get)
    val referenceMesh = MeshIO.readMesh(data.referenceMesh).get

    val dc = DataCollection.fromTriangleMesh3DSequence(referenceMesh, meshes)
    val alignedDC = DataCollection.gpa(dc)
    val pca = StatisticalMeshModel.createUsingPCA(alignedDC).get

    val ui = ScalismoUI()
    ui.show(pca, "model")

    StatisticalModelIO.writeStatisticalMeshModel(pca, data.pca)
  }
}
