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

import java.awt.Color
import java.io.File

import api.other.{ModelSampling, TargetSampling}
import api.registration.GpmmCpdRegistration
import api.registration.utils.modelViewer
import api.sampling.evaluators.SymmetricEvaluation
import api.sampling.{MixedProposalDistributions, ModelFittingParameters, ProductEvaluators, SamplingRegistration, ShapeParameters}
import apps.bfm.Paths.generalPath
import apps.util.RegistrationComparison
import breeze.linalg.DenseVector
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.geometry._3D
import scalismo.io.{LandmarkIO, MeshIO}
import scalismo.mesh.{TriangleMesh, TriangleMesh3D}
import scalismo.sampling.DistributionEvaluator
import scalismo.sampling.proposals.MixtureProposal
import scalismo.sampling.proposals.MixtureProposal.ProposalGeneratorWithTransition
import scalismo.statisticalmodel.{PointDistributionModel, StatisticalMeshModel}
import scalismo.ui.api.{ScalismoUI, ScalismoUIHeadless, StatisticalMeshModelViewControls}
import scalismo.utils.Random.implicits.randomGenerator

object BfmFittingThesis {

  def main(args: Array[String]) {
    scalismo.initialize()
    val completedPath = new File(generalPath, "completed")
    completedPath.mkdirs()
    val alignedPath = new File(generalPath, "aligned")
    val alignedMeshesPath = new File(alignedPath, "meshes")
    val fileList = alignedMeshesPath.listFiles().filter(f => f.getName.endsWith(".stl")).sorted.toIndexedSeq
    fileList.foreach(println(_))

//        val faceIndex = 7 // 0.2: 2, 4, 7, 0.1: 2, 6

    val faceIndexes = Seq(2,4,7)
    faceIndexes.par.foreach{ faceIndex =>
//    fileList.zipWithIndex.par.foreach { case (_, faceIndex) =>

      Thread.sleep(1000 * faceIndex)
      println(s"FACE INDEX: ${faceIndex}")

      // load the data
      val (modelInit, baseName, targetGroundTruth, targetMeshPartialInit, targetLogFile) = LoadTestData.modelAndTarget(faceIndex, 100)
      val model = modelInit.decimate(1000)
      val modelLms = LandmarkIO.readLandmarksJson3D(new File(generalPath, "bfm.json")).get
      val targetMeshPartial = targetMeshPartialInit.operations.decimate(1000)
      val decGPMM = model.decimate(model.rank * 2)
      val decTarget = targetMeshPartial.operations.decimate(model.rank * 2)
      println(s"Number of vertices in model: ${model.mean.pointSet.numberOfPoints}")
      val targetLms = LandmarkIO.readLandmarksJson3D(new File(generalPath, s"partial/landmarks/${baseName}.json")).get.filter(f => !f.id.contains("nose"))
      println("Target lms:")
      targetLms.foreach(f => println(f.id))
      // visualization setup
      val ui = ScalismoUI(s"BFM-icp-fitting ${faceIndex}")
//      val ui = ScalismoUIHeadless()

      val modelGroup = ui.createGroup("modelGroup")
      val targetGroup = ui.createGroup("targetGroup")
      val finalGroup = ui.createGroup("finalGroup")
      val gpmmView = ui.show(modelGroup, modelInit, "model")
      val showGt = ui.show(targetGroup, targetGroundTruth, "Ground-truth")
      showGt.opacity = 0.0
      val showTarget = ui.show(targetGroup, targetMeshPartialInit, "target")
      showTarget.color = Color.YELLOW

      // proposal

      val proposalCP1 = MixedProposalDistributions.mixedProposalICP(decGPMM, targetMeshPartial, modelLms, targetLms, projectionDirection = ModelSampling, tangentialNoise = 50.0, noiseAlongNormal = 5.0, stepLength = 0.1)
      val proposalCP2 = MixedProposalDistributions.mixedProposalICP(decGPMM, targetMeshPartial, modelLms, targetLms, projectionDirection = ModelSampling, tangentialNoise = 2.0, noiseAlongNormal = 1.0, stepLength = 0.5)
      val proposalCP3 = MixedProposalDistributions.mixedProposalICP(model, decTarget, modelLms, targetLms, projectionDirection = TargetSampling, tangentialNoise = 50.0, noiseAlongNormal = 5.0, stepLength = 0.1)
      val proposalCP4 = MixedProposalDistributions.mixedProposalICP(model, decTarget, modelLms, targetLms, projectionDirection = TargetSampling, tangentialNoise = 2.0, noiseAlongNormal = 1.0, stepLength = 0.5)
      val proposalPose = MixedProposalDistributions.randomPoseProposal(1.0)
      val proposalRandom = MixedProposalDistributions.mixedProposalRandom(decGPMM)

      //        val proposal = proposalCP
      val proposal = MixtureProposal.fromProposalsWithTransition(Seq((0.30, proposalCP1), (0.05, proposalCP2), (0.30, proposalCP3), (0.05, proposalCP4), (0.2, proposalPose), (0.1, proposalRandom)): _ *)


      // evaluator
      val numberOfEvaluationPoints = model.rank * 4
      val avgUncertainty = 0.2 // 0.2
      val maxUncertainty = 1.0
      val evaluator = ProductEvaluators.proximityAndCollectiveHausdorffBoundaryAware(
        model,
        targetMeshPartial,
        uncertaintyAvg = avgUncertainty,
        uncertaintyMax = maxUncertainty,
        mean = 0.0,
        numberOfEvaluationPoints = numberOfEvaluationPoints,
        evaluationMode = SymmetricEvaluation
      )

      // run the registration
      val numOfSamples = 10000

      def fitting(model: StatisticalMeshModel, targetMesh: TriangleMesh3D, evaluator: Map[String, DistributionEvaluator[ModelFittingParameters]], proposal: ProposalGeneratorWithTransition[ModelFittingParameters], numOfIterations: Int, showModel: Option[StatisticalMeshModelViewControls], log: File, initialParameters: Option[ModelFittingParameters] = None): ModelFittingParameters = {

        val samplingRegistration = new SamplingRegistration(model, targetMesh, showModel, modelUiUpdateInterval = 10, acceptInfoPrintInterval = 100)
        val t0 = System.currentTimeMillis()

        val best = samplingRegistration.runfitting(evaluator, proposal, numOfIterations, initialModelParameters = initialParameters, jsonName = log)

        val t1 = System.currentTimeMillis()
        println(s"ICP-Timing: ${(t1 - t0) / 1000.0} sec")
        best
      }

//      val mv = Option(modelViewer(gpmmView.shapeModelTransformationView, 10))
//
//      val pdm = PointDistributionModel[_3D, TriangleMesh](decGPMM.referenceMesh, decGPMM.gp.interpolate(NearestNeighborInterpolator()))
//      val gpmmCPD = new GpmmCpdRegistration[_3D, TriangleMesh](pdm, targetMeshPartial, modelLms, targetLms, lambda = 1, w = 0, max_iterations = 100, modelView = mv)
//      val cpdFit: DenseVector[Double] = gpmmCPD.register(tolerance = 0.0000001)
//
//      ui.show(finalGroup, model.instance(cpdFit), "cpd")
//
//      val initFitPars = ModelFittingParameters.zeroInitialization(modelInit).copy(shapeParameters = ShapeParameters(cpdFit))
//
//      val bestPars = fitting(decGPMM, decTarget, evaluator, proposal, numOfSamples, Option(gpmmView), targetLogFile, initialParameters = Option(initFitPars))
//
//      val bestRegistration = ModelFittingParameters.transformedMesh(model, bestPars)
//
//      RegistrationComparison.evaluateReconstruction2GroundTruthBoundaryAware(faceIndex.toString, bestRegistration, targetMeshPartial)
//
//      MeshIO.writeMesh(bestRegistration, new File(completedPath, faceIndex.toString + ".ply"))
    }
    println("Done now!")
  }
}
