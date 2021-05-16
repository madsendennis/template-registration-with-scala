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

import api.other.{ModelAndTargetSampling, ModelSampling, TargetSampling}
import api.sampling._
import api.sampling.evaluators.ModelToTargetEvaluation
import api.sampling.loggers.GiNGRExperimentLogger
import api.sampling.proposals.NonRigidCpdProposal
import apps.femur.Paths.{dataFemurPath, generalPath}
import apps.femur.RandomSamplesFromModel.InitialiseShapeParameters
import apps.util.FileUtils
import breeze.linalg.{DenseMatrix, DenseVector}
import scalismo.geometry._
import scalismo.io.{MeshIO, StatisticalModelIO}
import scalismo.mesh.{MeshMetrics, TriangleMesh, TriangleMesh3D}
import scalismo.sampling.proposals.MixtureProposal
import scalismo.statisticalmodel.{MultivariateNormalDistribution, StatisticalMeshModel}
import scalismo.ui.api.ScalismoUIHeadless
import scalismo.utils.Random.implicits._

object GiNGRvsPGiNGR_ComparisonAll {

  def appendExperiment(model: StatisticalMeshModel, log: GiNGRExperimentLogger, index: Int, target: TriangleMesh3D, bestSampleICP: TriangleMesh3D, bestSampleCPD: TriangleMesh3D, bestICP: TriangleMesh3D, bestCPD: TriangleMesh3D, targetPath: String, samplingICPLoggerPath: String, samplingCPDLoggerPath: String, coeffInit: DenseVector[Double], numOfEvaluationPoints: Int, numOfSamplePoints: Int, normalNoise: Double, comment: String): Unit = {
    def distMeasure(best: TriangleMesh3D): Map[String, Double] = {
      val avgDist1 = MeshMetrics.avgDistance(best, target)
      val avgDist2 = MeshMetrics.avgDistance(best, target)
      val avgDist = (avgDist1 + avgDist2) / 2.0
      val hausDist = MeshMetrics.hausdorffDistance(best, target)
      val diceCoeff = MeshMetrics.diceCoefficient(best, target)
      Map("avg" -> avgDist, "hausdorff" -> hausDist, "dice" -> diceCoeff)
    }

    val samplingICP = distMeasure(bestSampleICP)
    val samplingCPD = distMeasure(bestSampleCPD)
    val icp = distMeasure(bestICP)
    val cpd = distMeasure(bestCPD)
    val bestSampleCoeffICP = model.coefficients(bestSampleICP).toArray.toSeq
    val bestSampleCoeffCPD = model.coefficients(bestSampleCPD).toArray.toSeq
    val bestICPCoeff = model.coefficients(bestICP).toArray.toSeq
    val bestCPDCoeff = model.coefficients(bestCPD).toArray.toSeq
    log.append(index = index, targetPath = targetPath, samplingICPLoggerPath = samplingICPLoggerPath, samplingCPDLoggerPath = samplingCPDLoggerPath,
      coeffInit = coeffInit.toArray.toSeq, coeffSamplingICP = bestSampleCoeffICP, coeffSamplingCPD = bestSampleCoeffCPD, coeffICP = bestICPCoeff, coeffCPD = bestCPDCoeff,
      samplingICP = samplingICP, samplingCPD = samplingCPD, icp = icp, cpd = cpd, numOfEvaluationPoints = numOfEvaluationPoints, numOfSamplePoints = numOfSamplePoints, normalNoise = normalNoise, comment = comment)
    log.writeLog()
  }

  def randomInitPars(model: StatisticalMeshModel, index: Int): ModelFittingParameters = {
    val rotatCenter: EuclideanVector[_3D] = model.referenceMesh.pointSet.points.map(_.toVector).reduce(_ + _) * 1.0 / model.referenceMesh.pointSet.numberOfPoints.toDouble
    val initPoseParameters = PoseParameters(EuclideanVector3D(0, 0, 0), (0, 0, 0), rotationCenter = rotatCenter.toPoint)
    val initShapeParameters =
      if (index == 0) {
        println("Starting from mean")
        ShapeParameters(DenseVector.zeros[Double](model.rank))
      }
      else {
        val perturbationDistr = new MultivariateNormalDistribution(DenseVector.zeros(model.rank), DenseMatrix.eye[Double](model.rank) * 0.1)
        val rndsample = perturbationDistr.sample()
        ShapeParameters(rndsample)
      }
    ModelFittingParameters(initPoseParameters, initShapeParameters)
  }


  def main(args: Array[String]) {
    scalismo.initialize()

    println("Running GiNGR vs P-GiNGR with ICP and CPD")

    val normalNoise = 5.0
    val numOfSamples = 100
    val numOfIterations = 100
    val numOfEvalPoints = 400 // Used for the likelihood evaluator
    val numOfDecimatedPoints = 100 // Used for the ICP proposal

    val logPath = new File(dataFemurPath, "log/thesis_small_50_experiment/")
    logPath.mkdir()

    val modelFile = new File(dataFemurPath, "femur_gp_model_50-components.h5")
    val model = StatisticalModelIO.readStatisticalMeshModel(modelFile).get
    val modelDec = model.decimate(numOfDecimatedPoints)

    val subPath = "aligned"

    val targetMeshesInit = new File(generalPath, s"$subPath/meshes/").listFiles().filter(f => f.getName.endsWith(".stl")).sorted

    val experimentFile = new File(logPath, "experiments.json")
    val experimentLogger: GiNGRExperimentLogger = GiNGRExperimentLogger(experimentFile, modelFile.toString)

    println(s"Experiment file: ${experimentFile}")

//    val doneList = Seq(2,3,5,6,7,8,9,11,12,14,18,19,20,21,22,25,29,37,39,41,43,48,49).map(_.toString+".stl")
    val targetMeshes =  targetMeshesInit //.filter(f => !doneList.contains(f.getName))
    println(s"Number of targets: ${targetMeshes.length}")
    println("Target names:")
    targetMeshes.foreach(println(_))

    targetMeshes.zipWithIndex.par.foreach { case (targetMeshFile, _) =>
      println(s"Working with targetMesh: ${targetMeshFile.toString}")
      val targetMesh = MeshIO.readMesh(targetMeshFile).get
      val targetDec = targetMesh.operations.decimate(numOfDecimatedPoints)
      val basename = FileUtils.basename(modelFile)
      val targetname = FileUtils.basename(targetMeshFile)


      val innerLoop = (0 until 100)
      innerLoop.foreach { case (i) =>
        println(s"Starting fitting with random initialization of shape parameters: $i")

        val initPoseParameters = ModelFittingParameters.zeroInitialization(model)
        val initPars = initPoseParameters.copy(shapeParameters = InitialiseShapeParameters(model.rank, i))

        val samplingLoggerPathICP = new File(logPath, s"ICP-$normalNoise-$basename-$targetname-samples-$numOfSamples-$i-index.json")
        val samplingLoggerPathCPD = new File(logPath, s"CPD-$normalNoise-$basename-$targetname-samples-$numOfSamples-$i-index.json")

        // ########## ICP ########## //
        val bestICPpars = IcpRegistration.fitting(model, targetMesh, numOfDecimatedPoints, numOfIterations = numOfIterations, showModel = None, initialParameters = Option(initPars))
        val bestICP = ModelFittingParameters.transformedMesh(model, bestICPpars)

        // ########## CPD ########## //
        val bestCPDpars = CpdRegistration.fitting(modelDec, targetDec, numOfIterations = numOfIterations, mv = None, tolerance = 0.00001, initialParameters = Option(initPars))
        val bestCPD = ModelFittingParameters.transformedMesh(model, bestCPDpars)

        // ########## Sampling ICP ########## //
        val evaluator = ProductEvaluators.proximityAndIndependent(model.decimate(500), targetMesh, evaluationMode = ModelToTargetEvaluation, uncertainty = 2.0, numberOfEvaluationPoints = numOfEvalPoints)

        val proposalICP1 = MixedProposalDistributions.mixedProposalICP(model.decimate(100), targetMesh, Seq(),Seq(), projectionDirection = ModelSampling, tangentialNoise = 100.0, noiseAlongNormal = 5.0, stepLength = 0.5)
        val proposalICP2 = MixedProposalDistributions.mixedProposalICP(model, targetMesh.operations.decimate(100), Seq(),Seq(), projectionDirection = TargetSampling, tangentialNoise = 100.0, noiseAlongNormal = 5.0, stepLength = 0.5)
        val proposalICP = MixtureProposal.fromProposalsWithTransition(Seq((0.5, proposalICP1), (0.5, proposalICP2)): _ *)

        val bestSamplingParsICP = IcpProposalRegistration.fitting(model, targetMesh, evaluator, proposalICP, numOfSamples, None, samplingLoggerPathICP, initialParameters = Option(initPars))
        val bestSamplingICP = ModelFittingParameters.transformedMesh(model, bestSamplingParsICP)

        // ########## Sampling CPD ########## //
        val proposalCpd1 = NonRigidCpdProposal(modelDec, targetDec, stepLength = 0.5, generatedBy = "ShapeCpdProposal-step-0.5")
        val proposalCpd2 = NonRigidCpdProposal(modelDec, targetDec, stepLength = 0.1, generatedBy = "ShapeCpdProposal-step-0.1")
        val proposal = MixtureProposal.fromProposalsWithTransition(Seq((0.5, proposalCpd1), (0.5, proposalCpd2)): _ *)

        val bestSamplingParsCPD = CpdProposalRegistration.fitting(model, targetMesh, evaluator, proposal, numOfSamples, None, samplingLoggerPathCPD, initialParameters = Option(initPars))
        val bestSamplingCPD = ModelFittingParameters.transformedMesh(model, bestSamplingParsCPD)

        // ########## LOG IT ########## //
        appendExperiment(model, experimentLogger, i, targetMesh, bestSamplingICP, bestSamplingCPD, bestICP, bestCPD, targetMeshFile.toString, samplingLoggerPathICP.toString, samplingLoggerPathCPD.toString, initPars.shapeParameters.parameters, numOfEvalPoints, numOfSamples, normalNoise, "")
      }
    }
  }
}
