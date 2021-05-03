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

import api.other.{IcpBasedSurfaceFitting, ModelSampling, RegistrationComparison}
import api.registration.GpmmCpdRegistration
import api.registration.utils.modelViewer
import api.sampling._
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.geometry._3D
import scalismo.mesh.{TriangleMesh, TriangleMesh3D}
import scalismo.statisticalmodel.{PointDistributionModel, StatisticalMeshModel}
import scalismo.ui.api.{ScalismoUI, StatisticalMeshModelViewControls}

object CpdRegistration {

  def fitting(model: StatisticalMeshModel, targetMesh: TriangleMesh3D, numOfIterations: Int, mv: Option[modelViewer], tolerance: Double = 0.0001, initialParameters: Option[ModelFittingParameters] = None): ModelFittingParameters = {

    val initPars = initialParameters.getOrElse(ModelFittingParameters.zeroInitialization(model))

    val modelConvert = PointDistributionModel[_3D, TriangleMesh](model.referenceMesh, model.gp.interpolate(NearestNeighborInterpolator()))
    val gpmmCPD = new GpmmCpdRegistration[_3D, TriangleMesh](modelConvert, targetMesh, Seq(), Seq(), lambda = 1, w = 0, max_iterations = numOfIterations, modelView = mv)

    val t0 = System.currentTimeMillis()
    val bestPars = gpmmCPD.register(tolerance = tolerance, initialGPMM = initPars.shapeParameters.parameters)
    val t1 = System.currentTimeMillis()
    println(s"ICP-Timing: ${(t1 - t0) / 1000.0} sec")
    initPars.copy(shapeParameters = ShapeParameters(bestPars))
  }


  def main(args: Array[String]): Unit = {
    scalismo.initialize()

    println(s"Starting Standard NonRigid ICP registrations!")

    val (model, modelLms, targetMesh, targetLms) = LoadTestData.modelAndTarget()

    val numOfEvaluatorPoints = 100 // Used for the evaluation
    val numOfIterations = 100 // Number of ICP iterations

    val ui = ScalismoUI(s"CPD-registration")
    val modelGroup = ui.createGroup("modelGroup")
    val targetGroup = ui.createGroup("targetGroup")
    val finalGroup = ui.createGroup("finalGroup")

    val showModel = ui.show(modelGroup, model, "model")
    ui.show(modelGroup, modelLms, "landmarks")
    val showTarget = ui.show(targetGroup, targetMesh, "target")
    ui.show(targetGroup, targetLms, "landmarks")
    showTarget.color = Color.YELLOW

    val mv: Option[modelViewer] = Option(modelViewer(showModel.shapeModelTransformationView, 10))


    val bestPars = fitting(model.decimate(numOfEvaluatorPoints), targetMesh.operations.decimate(numOfEvaluatorPoints), numOfIterations = numOfIterations, mv = mv, tolerance = 0.00001)
    val bestRegistration = ModelFittingParameters.transformedMesh(model, bestPars)
    ui.show(finalGroup, bestRegistration, "best-fit")
    RegistrationComparison.evaluateReconstruction2GroundTruth("SAMPLE", bestRegistration, targetMesh)
  }
}
