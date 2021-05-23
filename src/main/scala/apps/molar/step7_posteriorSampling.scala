package apps.molar

import java.awt.Color
import java.io.File

import api.other.{ModelSampling, TargetSampling}
import api.registration.utils.{ClosestPointRegistrator, ReferenceToTarget, TargetToReference}
import api.sampling.evaluators.{ModelToTargetEvaluation, TargetToModelEvaluation}
import api.sampling.{MixedProposalDistributions, ModelFittingParameters, ProductEvaluators, ShapeParameters, SurfaceNoiseHelpers}
import api.sampling.loggers.JSONAcceptRejectLogger
import apps.femur.IcpProposalRegistration
import apps.util.{FileUtils, RegistrationComparison}
import breeze.linalg.{DenseMatrix, DenseVector}
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.io.{MeshIO, StatisticalModelIO}
import scalismo.sampling.proposals.MixtureProposal
import scalismo.statisticalmodel.{MultivariateNormalDistribution, StatisticalMeshModel}
import scalismo.ui.api.ScalismoUI
import scalismo.utils.Random.implicits.randomGenerator

object step7_posteriorSampling extends App {
  scalismo.initialize()

  val logPathInput = new File(data.log, "step5")
  val logFiles = logPathInput.listFiles(_.getName.endsWith(".json")).filter(!_.getName.contains("_init.json")).sorted
  println(s"LogPath: ${logPathInput}")

  val logPathOut = new File(data.log, "step7")
  logPathOut.mkdirs()

  val regMeshesAll = data.registeredMeshesCrownICP.listFiles(_.getName.endsWith(".ply")).sorted
  val targets = data.rawCrowns.listFiles(_.getName.endsWith(".stl")).sorted

  val regMeshes = regMeshesAll.take(10)

  val gpmm = StatisticalModelIO.readStatisticalTriangleMeshModel3D(data.augmentedAligned).get.truncate(800)
  val model = StatisticalMeshModel(gpmm.reference, gpmm.gp.interpolate(NearestNeighborInterpolator()))

  regMeshes.foreach{regInit =>
    val baseName = FileUtils.basename(regInit).replace("_icp", "")
    println(s"Processing: ${baseName}")
    val logFile = logFiles.find(f => FileUtils.basename(f) == baseName).get
    val targetFile = targets.find(f => FileUtils.basename(f) == baseName).get

    val initReg = MeshIO.readMesh(regInit).get
    val target = MeshIO.readMesh(targetFile).get

    val logObj = new JSONAcceptRejectLogger[ModelFittingParameters](logFile)
    val bestPars = logObj.getBestFittingParsFromJSON
    val trans = ModelFittingParameters.poseTransform(bestPars)
    val coeff = model.transform(trans).coefficients(initReg).slice(0,200)

    val ui = ScalismoUI(baseName)
    val modelGroup = ui.createGroup("model")
    val targetGroup = ui.createGroup("target")
    ui.show(targetGroup, target, "target").color = Color.YELLOW
    ui.show(targetGroup, initReg, "init").opacity = 0.0

    val model100 = model.truncate(200)

    val initPars = bestPars.copy(shapeParameters = ShapeParameters(coeff))
//    val initPars = ModelFittingParameters.zeroInitialization(model100).copy(shapeParameters = ShapeParameters(coeff))
    val numOfSamples = 1000
    val modelDec = model100.decimate(400)
    val targetDec = target.operations.decimate(400)
    val numOfEvalPoints = 100

    val log = new File(logPathOut, baseName+".json")

//    val evaluatorIcp = ProductEvaluators.proximityAndIndependent(model100.decimate(1000), target.operations.decimate(400), evaluationMode = TargetToModelEvaluation, uncertainty = 0.5, numberOfEvaluationPoints = numOfEvalPoints)

//    val evaluatorIcp = ProductEvaluators.proximityAndIndependentBoundary(model100.decimate(1000), target.operations.decimate(400), evaluationMode = TargetToModelEvaluation, uncertainty = 0.05, numberOfEvaluationPoints = numOfEvalPoints, boundaryAware = true)
//
//
//    val proposalIcp1 = MixedProposalDistributions.mixedProposalProjectionICP(modelDec, targetDec, Seq(), Seq(), projectionDirection = ModelSampling, tangentialNoise = 0.1, noiseAlongNormal = 0.1, stepLength = 0.1)
//    val proposalIcp2 = MixedProposalDistributions.mixedProposalProjectionICP(model100.decimate(400), targetDec, Seq(), Seq(), projectionDirection = TargetSampling, tangentialNoise = 0.1, noiseAlongNormal = 0.1, stepLength = 0.1)
//    val proposalIcp3 = MixedProposalDistributions.mixedProposalProjectionICP(modelDec, target.operations.decimate(400), Seq(), Seq(), projectionDirection = ModelSampling, tangentialNoise = 0.1, noiseAlongNormal = 0.1, stepLength = 1.0)
//    val proposalIcp4 = MixedProposalDistributions.mixedProposalProjectionICP(model100.decimate(400), targetDec, Seq(), Seq(), projectionDirection = TargetSampling, tangentialNoise = 0.1, noiseAlongNormal = 0.1, stepLength = 1.0)
//
//    val proposal = MixtureProposal.fromProposalsWithTransition(Seq((0.25, proposalIcp1), (0.25, proposalIcp2), (0.25, proposalIcp3), (0.25, proposalIcp4)): _ *)
//
//    val showModel = ui.show(modelGroup, model100, "model")
////    showModel.shapeModelTransformationView.poseTransformationView.transformation = trans
//    showModel.shapeModelTransformationView.shapeTransformationView.coefficients = coeff
//
//    println(s"${initPars.shapeParameters.parameters}")
//    println(s"${initPars.poseParameters.rotation}")
//    println(s"${initPars.poseParameters.translation}")
//    println(s"${initPars.poseParameters.rotationCenter}")
//
//    val bestSamplingPars = IcpProposalRegistration.fitting(modelDec, targetDec, evaluatorIcp, proposal, numOfSamples, Option(showModel), log, initialParameters = Option(initPars))

    val currentMesh = ModelFittingParameters.transformedMesh(model100, initPars)
    val cpInfo = ClosestPointRegistrator.ClosestPointTriangleMesh3D.closestPointCorrespondence(currentMesh, targetDec, direction = ReferenceToTarget)
    val cp = cpInfo._1.filter(_._3==1.0).toIndexedSeq


    val noiseAlongNormal = 0.1
    val tangentialNoise = 1.0
    val trainingData = cp.map{case(id, _, _) =>
      val noiseDistribution = SurfaceNoiseHelpers.surfaceNormalDependantNoise(currentMesh.vertexNormals.atPoint(id), noiseAlongNormal, tangentialNoise)
//      val noiseDistribution = MultivariateNormalDistribution(DenseVector.zeros[Double](3), DenseMatrix.eye[Double](3)*0.1)
      val p = currentMesh.pointSet.point(id)
      (id, p, noiseDistribution)
    }

//    val trainingData = cp.map{
//      case (id, p, _) =>
//        (id, currentMesh.pointSet.point(id), MultivariateNormalDistribution(DenseVector.zeros[Double](3), DenseMatrix.eye[Double](3)*0.1))
//    }
    val posModel = model100.transform(trans).posterior(trainingData)

    val posGroup = ui.createGroup("posterior")
    ui.show(posGroup, posModel, "pos")
    ui.show(targetGroup, currentMesh, "current").opacity = 0.0

  }
  println("All done")
}
