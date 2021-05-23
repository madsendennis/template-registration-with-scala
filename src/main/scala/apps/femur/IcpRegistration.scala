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

package apps.femur

import java.awt.Color

import api.other.{IcpBasedSurfaceFitting, ModelAndTargetSampling, ModelSampling, RegistrationComparison, TargetSampling}
import api.sampling._
import scalismo.geometry._3D
import scalismo.mesh.{TriangleMesh, TriangleMesh3D}
import scalismo.statisticalmodel.StatisticalMeshModel
import scalismo.ui.api.{ScalismoUI, StatisticalMeshModelViewControls}

object IcpRegistration {

  def fitting(model: StatisticalMeshModel, targetMesh: TriangleMesh3D, numOfSamplePoints: Int, numOfIterations: Int, showModel: Option[StatisticalMeshModelViewControls], initialParameters: Option[ModelFittingParameters] = None, iterationSeq: Seq[Double] = Seq(1e-15)): ModelFittingParameters = {

    val initPars =
      if (initialParameters.isDefined) {
        Option(initialParameters.get.shapeParameters.parameters, ModelFittingParameters.poseTransform(initialParameters.get))
      }
      else {
        None
      }
    val icpFitting = IcpBasedSurfaceFitting(model, targetMesh, numOfSamplePoints, projectionDirection = TargetSampling, showModel = showModel)
//    val icpFitting = IcpBasedSurfaceFitting(model, targetMesh, numOfSamplePoints, projectionDirection = ModelAndTargetSampling, showModel = showModel)
    val t0 = System.currentTimeMillis()

    val bestPars = icpFitting.runfitting(numOfIterations, iterationSeq = iterationSeq, initialModelParameters = initPars)
    val t1 = System.currentTimeMillis()
    println(s"ICP-Timing: ${(t1 - t0) / 1000.0} sec")
    val pars = initialParameters.getOrElse(ModelFittingParameters.zeroInitialization(model))
    pars.copy(shapeParameters = ShapeParameters(bestPars))
  }


  def main(args: Array[String]): Unit = {
    scalismo.initialize()

    println(s"Starting Standard NonRigid ICP registrations!")

    val (model, modelLms, targetMesh, targetLms) = LoadTestData.modelAndTarget()

    val numOfEvaluatorPoints = 100 // Used for the evaluation
    val numOfIterations = 100 // Number of ICP iterations

    val ui = ScalismoUI(s"ICP-registration")
    val modelGroup = ui.createGroup("modelGroup")
    val targetGroup = ui.createGroup("targetGroup")
    val finalGroup = ui.createGroup("finalGroup")

    val showModel = ui.show(modelGroup, model, "model")
    ui.show(modelGroup, modelLms, "landmarks")
    val showTarget = ui.show(targetGroup, targetMesh, "target")
    ui.show(targetGroup, targetLms, "landmarks")
    showTarget.color = Color.YELLOW

    val bestPars = fitting(model, targetMesh, numOfEvaluatorPoints, numOfIterations = numOfIterations, showModel = Option(showModel))
    val bestRegistration = ModelFittingParameters.transformedMesh(model, bestPars)
    ui.show(finalGroup, bestRegistration, "best-fit")
    RegistrationComparison.evaluateReconstruction2GroundTruth("SAMPLE", bestRegistration, targetMesh)
  }
}
