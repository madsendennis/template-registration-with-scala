package apps.femurCourse

import java.awt.Color
import java.io.File

import api.other.{RegistrationComparison, TargetSampling}
import api.registration.utils.modelViewer
import api.sampling.evaluators.TargetToModelEvaluation
import api.sampling.{MixedProposalDistributions, ModelFittingParameters, ProductEvaluators, SamplingRegistration}
import apps.util.{AlignmentTransforms, FileUtils}
import scalismo.geometry.{Point3D, _3D}
import scalismo.io.{LandmarkIO, MeshIO, StatisticalModelIO}
import scalismo.mesh.{TriangleMesh, TriangleMesh3D}
import scalismo.sampling.DistributionEvaluator
import scalismo.sampling.proposals.MixtureProposal
import scalismo.sampling.proposals.MixtureProposal.ProposalGeneratorWithTransition
import scalismo.statisticalmodel.StatisticalMeshModel
import scalismo.ui.api.{ScalismoUI, ScalismoUIHeadless, StatisticalMeshModelViewControls}
import scalismo.utils.Random.implicits.randomGenerator

object step10_visualizePreg extends App {
//  scalismo.initialize()
//
//  val logPath = new File(data.data, "log")
//  logPath.mkdirs()
//
//  val completedPath = new File(data.completed, "preg_pca")
//  completedPath.mkdirs()
//
//  //  val modelInit = StatisticalModelIO.readStatisticalTriangleMeshModel3D(data.pca).get
//  val modelInit = StatisticalModelIO.readStatisticalMeshModel(data.pca).get
//
//  val modelLmsInit = LandmarkIO.readLandmarksJson3D(data.referenceLms).get
//
//  val targetFiles = data.step3.listFiles(f => f.getName.endsWith(".stl")).sorted
//
//  //    val targetFile = targetFiles(1) //.find(f => f.getName.equals("VSD.case_2.101148.stl")).get
//  targetFiles.par.foreach { targetFile =>
//
//    println(s"Processing: ${targetFile}")
//    val baseName = FileUtils.basename(targetFile)
//    val target = MeshIO.readMesh(targetFile).get
//    val targetLms = LandmarkIO.readLandmarksJson3D(new File(targetFile.getParent, baseName + ".json")).get
//
//    println(s"Target points: ${target.pointSet.numberOfPoints}")
//    val transform = AlignmentTransforms.computeTransform(modelLmsInit, targetLms, Point3D(0, 0, 0))
//    val gpmm = modelInit.transform(transform)
//    val modelLms = modelLmsInit.map(_.transform(transform))
//
//    //    val commonLmNamesInit = modelLms.map(_.id) intersect targetLms.map(_.id)
//    //    val commonLmNames = commonLmNamesInit.filter(f => !f.toLowerCase.contains("shaft"))
//    //    val trainingData = commonLmNames.map(name => (gpmm.mean.pointSet.findClosestPoint(modelLms.find(_.id == name).get.point).id, targetLms.find(_.id == name).get.point)).toIndexedSeq
//    //    val gpmmPos = gpmm.posterior(trainingData, 100.0)
//
//    val ui: ScalismoUIHeadless = ScalismoUIHeadless()
//    //    val ui: ScalismoUI = ScalismoUI()
//    val modelGroup = ui.createGroup("gpmm")
//    val targetGroup = ui.createGroup("target")
//    val gpmmView = ui.show(modelGroup, gpmm, "model")
//    val showTarget = ui.show(targetGroup, target, targetFile.getName)
//    showTarget.color = Color.YELLOW
//    //    ui.show(targetGroup, targetLms, "lms")
//    //    ui.show(modelGroup, modelLms, "lms")
//
//    val mv = if (ui.isInstanceOf[ScalismoUI]) Option(modelViewer(gpmmView.shapeModelTransformationView, 10)) else None
//
//    val numOfSamples = 10000 // Length of Markov Chain
//    val samplePointsModel = 5000
//    val samplePointsTarget = 1000
//    val decTarget = target.operations.decimate(samplePointsTarget)
//    val decGPMM = gpmm.decimate(samplePointsModel)
//    val proposalCP = MixedProposalDistributions.mixedProposalICP(decGPMM, decTarget, Seq(), Seq(), samplePointsTarget, projectionDirection = TargetSampling, tangentialNoise = 100.0, noiseAlongNormal = 5.0, stepLength = 0.5)
//    val proposalPose = MixedProposalDistributions.randomPoseProposal(1.0)
//
//    //    val proposal = proposalCP
//    val proposal = MixtureProposal.fromProposalsWithTransition(Seq((0.5, proposalCP), (0.5, proposalPose)): _ *)
//
//    val evaluator = ProductEvaluators.proximityAndIndependent(decGPMM, decTarget, evaluationMode = TargetToModelEvaluation, uncertainty = 2.0, numberOfEvaluationPoints = samplePointsTarget)
//
//
//    def fitting(model: StatisticalMeshModel, targetMesh: TriangleMesh3D, evaluator: Map[String, DistributionEvaluator[ModelFittingParameters]], proposal: ProposalGeneratorWithTransition[ModelFittingParameters], numOfIterations: Int, showModel: Option[StatisticalMeshModelViewControls], log: File, initialParameters: Option[ModelFittingParameters] = None): TriangleMesh[_3D] = {
//
//      val samplingRegistration = new SamplingRegistration(model, targetMesh, showModel, modelUiUpdateInterval = 10, acceptInfoPrintInterval = 100)
//      val t0 = System.currentTimeMillis()
//
//      val best = samplingRegistration.runfitting(evaluator, proposal, numOfIterations, initialModelParameters = initialParameters, jsonName = log)
//
//      val t1 = System.currentTimeMillis()
//      println(s"ICP-Timing: ${(t1 - t0) / 1000.0} sec")
//      ModelFittingParameters.transformedMesh(model, best)
//    }
//
//    StatisticalModelIO.writeStatisticalMeshModel(gpmm, new File(logPath, s"${baseName}.h5"))
//    val bestRegistration = fitting(decGPMM, decTarget, evaluator, proposal, numOfSamples, Option(gpmmView), new File(logPath, s"${baseName}.json"))
//
//    RegistrationComparison.evaluateReconstruction2GroundTruth(baseName, target, bestRegistration)
//
//    MeshIO.writeMesh(bestRegistration, new File(completedPath, targetFile.getName))
//  }
}
