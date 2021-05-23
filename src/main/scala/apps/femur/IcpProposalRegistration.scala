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
import java.io.File

import api.other.{ModelAndTargetSampling, ModelSampling, RegistrationComparison, TargetSampling}
import api.sampling._
import api.sampling.evaluators.{ModelToTargetEvaluation, SymmetricEvaluation}
import apps.femur.Paths.dataFemurPath
import scalismo.geometry._3D
import scalismo.mesh.{TriangleMesh, TriangleMesh3D}
import scalismo.sampling.DistributionEvaluator
import scalismo.sampling.proposals.MixtureProposal
import scalismo.sampling.proposals.MixtureProposal.ProposalGeneratorWithTransition
import scalismo.statisticalmodel.StatisticalMeshModel
import scalismo.ui.api.{ScalismoUI, StatisticalMeshModelViewControls}
import scalismo.utils.Random.implicits.randomGenerator

object IcpProposalRegistration {

  def fitting(model: StatisticalMeshModel, targetMesh: TriangleMesh3D, evaluator: Map[String, DistributionEvaluator[ModelFittingParameters]], proposal: ProposalGeneratorWithTransition[ModelFittingParameters], numOfIterations: Int, showModel: Option[StatisticalMeshModelViewControls], log: File, initialParameters: Option[ModelFittingParameters] = None): ModelFittingParameters = {

    val samplingRegistration = new SamplingRegistration(model, targetMesh, showModel, modelUiUpdateInterval = 1, acceptInfoPrintInterval = 20)
    val t0 = System.currentTimeMillis()

    val best = samplingRegistration.runfitting(evaluator, proposal, numOfIterations, initialModelParameters = initialParameters, jsonName = log)

    val t1 = System.currentTimeMillis()
    println(s"ICP-Timing: ${(t1 - t0) / 1000.0} sec")
    best
  }

  def main(args: Array[String]): Unit = {
    scalismo.initialize()

    println(s"Starting Metropolis Hastings registrations with ICP-proposal!")

    val logPath = new File(dataFemurPath, "log")

    val (model, modelLms, targetMesh, targetLms) = LoadTestData.modelAndTarget()

    val numOfEvaluatorPoints = model.referenceMesh.pointSet.numberOfPoints/2 // Used for the likelihood evaluator
    val numOfICPPointSamples = model.rank*2 // Used for the ICP proposal
    val numOfSamples = 200 // Length of Markov Chain

    /***** ***** ***** ***** ***** *****
    * Closest Point proposal configuration
    *  projectionDirection:
    *  - TargetSampling (if registering partial meshes)
    *  - ModelSampling (if registering noisy meshes)
    *  - ModelAndTargetSampling (if registering clean complete meshes)
    ***** ***** ***** ***** ***** *****/
    val proposal1 = MixedProposalDistributions.mixedProposalICP(model.decimate(100), targetMesh, Seq(),Seq(), projectionDirection = ModelSampling, tangentialNoise = 100.0, noiseAlongNormal = 5.0, stepLength = 0.5)
    val proposal2 = MixedProposalDistributions.mixedProposalICP(model, targetMesh.operations.decimate(100), Seq(),Seq(), projectionDirection = TargetSampling, tangentialNoise = 100.0, noiseAlongNormal = 5.0, stepLength = 0.5)

    val proposal = MixtureProposal.fromProposalsWithTransition(Seq((0.5, proposal1), (0.5, proposal2)): _ *)
    /* Uncomment below to use the standard "Random walk proposal" proposal */
//    val proposal = MixedProposalDistributions.mixedProposalRandom(model)

    /***** ***** ***** ***** ***** *****
    * Choosing the likelihood function
    *  - euclideanEvaluator (gaussian distribution): gives best L2 distance restults
    *  - hausdorffEvaluator (exponential distribution): gives best hausdorff result
    * evaluationMode:
    *  - ModelToTargetEvaluation (if registering noisy meshes)
    *  - TargetToModelEvaluation (if registering partial meshes)
    *  - SymmetricEvaluation (if registering clean complete meshes)
    ***** ***** ***** ***** ***** *****/
    val evaluator = ProductEvaluators.proximityAndIndependent(model.decimate(500), targetMesh, evaluationMode = ModelToTargetEvaluation, uncertainty = 2.0, numberOfEvaluationPoints = numOfEvaluatorPoints)
    /* Uncomment below to use the hausdorff likelihood function */
//    val evaluator = ProductEvaluators.proximityAndHausdorff(model, targetMesh, uncertainty = 100.0)

    val ui = ScalismoUI(s"MH-ICP-proposal-registration")
    val modelGroup = ui.createGroup("modelGroup")
    val targetGroup = ui.createGroup("targetGroup")
    val finalGroup = ui.createGroup("finalGroup")

    val showModel = ui.show(modelGroup, model, "model")
    ui.show(modelGroup, modelLms, "landmarks")
    val showTarget = ui.show(targetGroup, targetMesh, "target")
    ui.show(targetGroup, targetLms, "landmarks")
    showTarget.color = Color.YELLOW

    val bestPars = fitting(model, targetMesh, evaluator, proposal, numOfSamples, Option(showModel), new File(logPath, s"icpProposalRegistration.json"))
    val bestRegistration = ModelFittingParameters.transformedMesh(model, bestPars)
    ui.show(finalGroup, bestRegistration, "best-fit")
    RegistrationComparison.evaluateReconstruction2GroundTruth("SAMPLE", bestRegistration, targetMesh)
  }
}
