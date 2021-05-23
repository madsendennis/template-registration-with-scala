package apps.molar

import java.io.File

import api.other.{ModelSampling, TargetSampling}
import api.sampling.evaluators.TargetToModelEvaluation
import api.sampling.{MixedProposalDistributions, ModelFittingParameters, ProductEvaluators, ShapeParameters}
import apps.femur.{IcpProposalRegistration, IcpRegistration}
import apps.util.{FileUtils, RegistrationComparison}
import breeze.linalg.DenseVector
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.io.{MeshIO, StatisticalModelIO}
import scalismo.sampling.proposals.MixtureProposal
import scalismo.statisticalmodel.StatisticalMeshModel
import scalismo.ui.api.{ScalismoUI, ScalismoUIHeadless}
import scalismo.utils.Random.implicits.randomGenerator

object step5_crownFitting extends App {
  scalismo.initialize()

  val logPath = new File(data.log, "step5")
  logPath.mkdirs()

  val completedPath = data.registeredMeshesCrownICP
  completedPath.mkdirs()

  val alreadyDone = completedPath.listFiles(f => f.getName.endsWith("_icp.ply")).map(f => FileUtils.basename(f).replace("_icp", "")).sorted

  val meshes = data.rawCrowns.listFiles(_.getName.endsWith(".stl")).sorted.filter(f => !alreadyDone.contains(FileUtils.basename(f)))

  println("already done:")
  alreadyDone.foreach(println)
  println("todo:")
  meshes.foreach(println)

  //  val targetFile = meshes(2)

  meshes.par.take(30).foreach { targetFile =>
    println(s"Processing: ${targetFile}")
    val gpmm = StatisticalModelIO.readStatisticalTriangleMeshModel3D(data.augmentedAligned).get.truncate(800)
    val model = StatisticalMeshModel(gpmm.reference, gpmm.gp.interpolate(NearestNeighborInterpolator()))
    println(s"GPMM rank: ${gpmm.rank}, ref points: ${gpmm.reference.pointSet.numberOfPoints}")

    val target = MeshIO.readMesh(targetFile).get
    println(s"Target points: ${target.pointSet.numberOfPoints}")
    val baseName = FileUtils.basename(targetFile)

    val ui = ScalismoUIHeadless()
//    val ui = ScalismoUI()
    val modelGroup = ui.createGroup("model")
    val modelGroup100 = ui.createGroup("model100")
    val targetGroup = ui.createGroup("target")
    //    val gpmmView = ui.show(modelGroup, gpmm, "model")
    ui.show(targetGroup, target, targetFile.getName)

    //    val mv = Option(modelViewer(gpmmView.shapeModelTransformationView, 10))

    try {

      val model100 = model.truncate(100)
      val model100dec = model100.decimate(200)
      val targetDec = target.operations.decimate(200)
      val numOfEvalPointsInit = model100.rank

      val modelView100 = ui.show(modelGroup100, model100, "model")

      val numOfInitSamples = 1000
      val numOfSamples = 500


      val dummyLog = new File(logPath, s"${baseName}_init.json")
      val log = new File(logPath, s"${baseName}.json")


      println("Computing prefitting")

      val evaluatorInit = ProductEvaluators.proximityAndIndependent(model100.decimate(500), target.operations.decimate(500), evaluationMode = TargetToModelEvaluation, uncertainty = 0.1, numberOfEvaluationPoints = numOfEvalPointsInit)
      val proposalPoseInit = MixedProposalDistributions.myRandomPoseProposalMolarInit() //(1.0)
      val proposalRandom = MixedProposalDistributions.mixedProposalRandomMolar(model100)

      val initPars100 = ModelFittingParameters.zeroInitialization(model100)

      val mixInit = MixtureProposal.fromProposalsWithTransition(Seq((0.80, proposalPoseInit), (0.20, proposalRandom)): _ *)
      val bestSamplingParsInit100 = IcpProposalRegistration.fitting(model100dec, targetDec, evaluatorInit, mixInit, numOfInitSamples, Option(modelView100), dummyLog, initialParameters = Option(initPars100))

      val proposalIcp1 = MixedProposalDistributions.mixedProposalICP(model100dec, target.operations.decimate(1000), Seq(), Seq(), projectionDirection = ModelSampling, tangentialNoise = 3.0, noiseAlongNormal = 0.3, stepLength = 0.1)
      val proposalIcp2 = MixedProposalDistributions.mixedProposalICP(model100.decimate(400), targetDec, Seq(), Seq(), projectionDirection = TargetSampling, tangentialNoise = 3.0, noiseAlongNormal = 0.3, stepLength = 0.1)
      val proposalIcp3 = MixedProposalDistributions.mixedProposalICP(model100dec, target.operations.decimate(1000), Seq(), Seq(), projectionDirection = ModelSampling, tangentialNoise = 5.0, noiseAlongNormal = 0.5, stepLength = 0.5)
      val proposalIcp4 = MixedProposalDistributions.mixedProposalICP(model100.decimate(400), targetDec, Seq(), Seq(), projectionDirection = TargetSampling, tangentialNoise = 5.0, noiseAlongNormal = 0.5, stepLength = 0.5)

      val proposalPose = MixedProposalDistributions.myRandomPoseProposalMolar()

      val proposal = MixtureProposal.fromProposalsWithTransition(Seq((0.20, proposalIcp1), (0.20, proposalIcp2), (0.05, proposalIcp3), (0.05, proposalIcp4), (0.25, proposalPose), (0.25, proposalRandom)): _ *)
      val evaluatorIcp = ProductEvaluators.proximityAndIndependent(model100.decimate(2000), target.operations.decimate(500), evaluationMode = TargetToModelEvaluation, uncertainty = 0.1, numberOfEvaluationPoints = numOfEvalPointsInit)

      val bestSamplingPars = IcpProposalRegistration.fitting(model100dec, targetDec, evaluatorIcp, proposal, numOfSamples, Option(modelView100), log, initialParameters = Option(bestSamplingParsInit100))

      val trans = ModelFittingParameters.poseTransform(bestSamplingPars)

      modelView100.meshView.opacity = 0.0
      val modelTrans = model.truncate(800).transform(trans)
      val modelView = ui.show(modelGroup, modelTrans, "model")
      println(s"${model100.rank}, ${modelTrans.rank}, ${bestSamplingPars.shapeParameters.parameters.length}")
      val shapePars = DenseVector.vertcat(bestSamplingPars.shapeParameters.parameters, DenseVector.zeros[Double](modelTrans.rank - model100.rank))


      val samplingConverted = ModelFittingParameters.zeroInitialization(modelTrans).copy(shapeParameters = ShapeParameters(shapePars))


      modelView.shapeModelTransformationView.shapeTransformationView.coefficients = samplingConverted.shapeParameters.parameters
      //    modelView.shapeModelTransformationView.poseTransformationView.transformation = ModelFittingParameters.poseTransform(samplingConverted)

      val bestICPpars = IcpRegistration.fitting(modelTrans.decimate(modelTrans.rank * 4), target, numOfSamplePoints = modelTrans.rank, numOfIterations = 20, showModel = Option(modelView), initialParameters = Option(samplingConverted), iterationSeq = Seq(0.1, 0.05))
      val fitMesh = ModelFittingParameters.transformedMesh(modelTrans, bestICPpars)

      RegistrationComparison.evaluateReconstruction2GroundTruthDouble(targetFile.getName, fitMesh, target)

      MeshIO.writeMesh(fitMesh, new File(completedPath, baseName + "_icp.ply"))
    }
    catch {
      case _: Throwable => println(s"Error with ${targetFile}")
    }
  }
}

// std icp ID: 0.stl average2surface: 0.30686905707228307 hausdorff: 2.9000570990369376
// norm icp ID:
//ID: 16.stl average2surface: 0.31717488278389255 hausdorff: 2.769287644764168