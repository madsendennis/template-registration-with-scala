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

import apps.util.AlignmentTransforms
import scalismo.common.EuclideanSpace
import scalismo.common.interpolation.TriangleMeshInterpolator3D
import scalismo.geometry._
import scalismo.io.{LandmarkIO, StatisticalModelIO}
import scalismo.kernels._
import scalismo.statisticalmodel.{GaussianProcess, LowRankGaussianProcess, StatisticalMeshModel}
import scalismo.ui.api.ScalismoUI
import scalismo.utils.Random


object fixModelOffset {
  implicit val random: Random = Random(1024)

  def main(args: Array[String]): Unit = {
    scalismo.initialize()

    val augModel = StatisticalModelIO.readStatisticalMeshModel(data.augmented).get
    val modelLms = LandmarkIO.readLandmarksJson3D(data.referenceLms).get

    val targetLms = LandmarkIO.readLandmarksJson3D(new File("/Volumes/storage/Dropbox/Workspace/uni-data/albert/surface/trainingsets_teeth/uk6er_extended/0012_36.json")).get

    val trans = AlignmentTransforms.computeTransform(modelLms, targetLms, Point3D(0,0,0))

    StatisticalModelIO.writeStatisticalMeshModel(augModel.transform(trans), data.augmentedAligned)
  }
}
