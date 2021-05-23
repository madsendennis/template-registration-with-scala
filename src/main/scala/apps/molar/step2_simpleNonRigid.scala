package apps.molar

import java.io.File

import api.other.{ModelSampling, TargetSampling}
import api.sampling.evaluators.ModelToTargetEvaluation
import api.sampling.proposals.NonRigidCpdProposal
import api.sampling.{MixedProposalDistributions, ModelFittingParameters, ProductEvaluators}
import apps.femur.CpdProposalRegistration
import apps.util.{FileUtils, RegistrationComparison}
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.io.{MeshIO, StatisticalModelIO}
import scalismo.sampling.proposals.MixtureProposal
import scalismo.statisticalmodel.StatisticalMeshModel
import scalismo.ui.api.{ScalismoUI, ScalismoUIHeadless}
import scalismo.utils.Random.implicits.randomGenerator

object step2_simpleNonRigid extends App {
  scalismo.initialize()

  val logPath = new File(data.log, "step2")
  logPath.mkdirs()

  data.registeredMeshesCoarse.mkdirs()

  val alreadyDone = data.registeredMeshesCoarse.listFiles(_.getName.endsWith(".ply")).map(_.getName)

  val meshes = data.alignedMeshesComplete.listFiles(_.getName.endsWith(".ply")).sorted.filter(f => !alreadyDone.contains(f.getName))

//    val targetFile = meshes(0) //.find(f => f.getName.equals("2.11.89.ply")).get
  println("Already done:")
  alreadyDone.foreach(println)
  println("Still missing: ")
  meshes.foreach(println)

  val status = meshes.zipWithIndex.par.map { case(targetFile, i) =>
    val gpmm = StatisticalModelIO.readStatisticalTriangleMeshModel3D(data.gpmmCoarse).get
    println(s"GPMM rank: ${gpmm.rank}, ref points: ${gpmm.reference.pointSet.numberOfPoints}")
    val model = StatisticalMeshModel(gpmm.reference, gpmm.gp.interpolate(NearestNeighborInterpolator()))
    val modelDec = model.decimate(model.rank * 2)
    println(s"Processing: ${targetFile}")
    val target = MeshIO.readMesh(targetFile).get
    val baseName = FileUtils.basename(targetFile)
    println(s"Target points: ${target.pointSet.numberOfPoints}")
    val targetDec = target.operations.decimate(model.rank * 2)
//    val ui = ScalismoUIHeadless()
    val ui = ScalismoUI()
    val modelGroup = ui.createGroup("gpmm")
    val targetGroup = ui.createGroup("target")
    val finalGroup = ui.createGroup("final")
    //    val gpmmView = ui.show(modelGroup, gpmm, "model")
    val modelView = ui.show(modelGroup, model, "model")

    ui.show(targetGroup, target, targetFile.getName)

    //    val mv = Option(modelViewer(gpmmView.shapeModelTransformationView, 10))

    val inTheEnd = try {
    //        val decGPMMcpd = gpmm.newReference(gpmm.reference.operations.decimate(600), NearestNeighborInterpolator())
    //        val gpmmCPD = new GpmmCpdRegistration[_3D, TriangleMesh](decGPMMcpd, target.operations.decimate(2000), Seq(), Seq(), lambda = 1, w = 0, max_iterations = 47, modelView = mv)
    //        val cpdFit = gpmmCPD.register(tolerance = 0.0001)
    //
    //        RegistrationComparison.evaluateReconstruction2GroundTruthDouble(targetFile.getName, gpmm.instance(cpdFit), target)
    ////
    //        val decGPMMicp = gpmm.newReference(gpmm.reference.operations.decimate(5000), NearestNeighborInterpolator())
    //        val gpmmICP = new NonRigidICPwithGPMMTriangle3D(decGPMMicp, target.operations.decimate(10000), mv)
    //        val icpFit = gpmmICP.Registration(max_iteration = 10, tolerance = 0.000000001, sigma2 = Seq(2.0, 1.0), initialGPMM = cpdFit)
    //        val fitMesh = gpmm.instance(icpFit)

    val initPars = ModelFittingParameters.zeroInitialization(model)
    val log = new File(logPath, s"${baseName}.sjon")
    val numOfEvalPoints = model.rank * 4
    val evaluator = ProductEvaluators.proximityAndIndependent(model.decimate(500), target, evaluationMode = ModelToTargetEvaluation, uncertainty = 0.1, numberOfEvaluationPoints = numOfEvalPoints)

    val numOfSamples = 1000
    val proposalCpd1 = NonRigidCpdProposal(modelDec, targetDec, stepLength = 0.5, generatedBy = "ShapeCpdProposal-step-0.5")
    val proposalCpd2 = NonRigidCpdProposal(modelDec, targetDec, stepLength = 0.1, generatedBy = "ShapeCpdProposal-step-0.1")
    val proposalIcp1 = MixedProposalDistributions.mixedProposalICP(modelDec, target.operations.decimate(500), Seq(), Seq(), projectionDirection = ModelSampling, tangentialNoise = 1.0, noiseAlongNormal = 0.2, stepLength = 0.2)
    val proposalIcp2 = MixedProposalDistributions.mixedProposalICP(model.decimate(500), targetDec, Seq(), Seq(), projectionDirection = TargetSampling, tangentialNoise = 1.0, noiseAlongNormal = 0.2, stepLength = 0.2)

    val proposalPose = MixedProposalDistributions.myRandomPoseProposalMolar() //(1.0)
    val proposalRandom = MixedProposalDistributions.mixedProposalRandomMolar(model)
    val proposal = MixtureProposal.fromProposalsWithTransition(Seq((0.10, proposalIcp1), (0.10, proposalIcp2), (0.15, proposalCpd1), (0.15, proposalCpd2), (0.25, proposalPose), (0.25, proposalRandom)): _ *)

    val bestSamplingParsCPD = CpdProposalRegistration.fitting(model, target, evaluator, proposal, numOfSamples, Option(modelView), log, initialParameters = Option(initPars))
    val fitMesh = ModelFittingParameters.transformedMesh(model, bestSamplingParsCPD)

    ui.show(finalGroup, fitMesh, "fit")

    RegistrationComparison.evaluateReconstruction2GroundTruthDouble(targetFile.getName, fitMesh, target)

    MeshIO.writeMesh(fitMesh, new File(data.registeredMeshesCoarse, targetFile.getName))
    targetFile.getName + "All good"
        }
        catch{
            case _: Throwable => {println(s"Error with ${targetFile}"); targetFile.getName + "Error"}
        }
    inTheEnd
  }
  println("Final:")
  status.foreach(println)
}

// std icp ID: 0.stl average2surface: 0.30686905707228307 hausdorff: 2.9000570990369376
// norm icp ID:
//ID: 16.stl average2surface: 0.31717488278389255 hausdorff: 2.769287644764168