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

import scalismo.common.interpolation.TriangleMeshInterpolator3D
import scalismo.common.{EuclideanSpace, Field}
import scalismo.geometry._
import scalismo.io.{LandmarkIO, StatisticalModelIO}
import scalismo.kernels._
import scalismo.statisticalmodel.{GaussianProcess, LowRankGaussianProcess, StatisticalMeshModel}
import scalismo.ui.api.ScalismoUI
import scalismo.utils.Random


object step4_createAugmentedModel {
  implicit val random: Random = Random(1024)

  def main(args: Array[String]): Unit = {
    scalismo.initialize()

    val pcaModel: StatisticalMeshModel = StatisticalModelIO.readStatisticalMeshModel(data.pca).get
    val referenceMesh = pcaModel.referenceMesh

    println("Num of points in ref: " + referenceMesh.pointSet.numberOfPoints)

    case class ChangePointKernel(kernel1: MatrixValuedPDKernel[_3D], center: Point3D, radius: Double)
      extends MatrixValuedPDKernel[_3D]() {

      override def domain: EuclideanSpace[_3D] = EuclideanSpace[_3D]

      val outputDim = 3

      def s(p: Point[_3D]) = {
//        val distance = Math.max(0, Math.sqrt(Math.pow(p.x - center.x, 2) + Math.pow(p.y - center.y, 2) + Math.pow(p.z - center.z, 2)) - radius)
        val distance = radius-(center.toVector-p.toVector).norm*0.2
//        1.0 / (1 + math.exp(-1+distance))
        math.max(0.0, math.min(distance, 1.0))
      }

      def k(x: Point[_3D], y: Point[_3D]) = {
        val sx = s(x)
        val sy = s(y)
        kernel1(x, y) * sx * sy //+ kernel2(x,y) * (1-sx) * (1-sy)
      }
    }

    val modelLms = LandmarkIO.readLandmarksJson[_3D](data.referenceLms).get
    val centerLandmark = modelLms.find(_.id == "MiddlePit").get.point

    val myKernel = DiagonalKernel(GaussianKernel[_3D](6) * 0.3, 3) +
      DiagonalKernel(GaussianKernel[_3D](3) * 0.3, 3) +
      DiagonalKernel(GaussianKernel[_3D](1) * 0.1, 3) +
      DiagonalKernel(GaussianKernel[_3D](0.5) * 0.05, 3)
    val cpKernel = ChangePointKernel(myKernel, centerLandmark, 1.1)

    val gp = GaussianProcess[_3D, EuclideanVector[_3D]](cpKernel)
    val lowRankGP: LowRankGaussianProcess[_3D, EuclideanVector[_3D]] = LowRankGaussianProcess.approximateGPCholesky(referenceMesh, gp, relativeTolerance = 0.01, interpolator = TriangleMeshInterpolator3D[EuclideanVector[_3D]]())

    val augmentedModel = StatisticalMeshModel.augmentModel(pcaModel, lowRankGP)
//    val augmentedModel = StatisticalMeshModel(referenceMesh, lowRankGP)

    val rank = augmentedModel.rank

    val ui = ScalismoUI()

    println(augmentedModel.gp.klBasis.map(_.eigenvalue))

    val modelGroup = ui.createGroup(s"Model-$rank")
    ui.show(modelGroup, augmentedModel, "model")

    println(s"Writing Augmented PCA model to: ${data.augmented}")
    StatisticalModelIO.writeStatisticalMeshModel(augmentedModel, data.augmented)
  }
}
