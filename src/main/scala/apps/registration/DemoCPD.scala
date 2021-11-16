package apps.registration

import api.{GeneralRegistrationState, GlobalTranformationType, NoTransforms, RigidTransforms}
import api.registration.config.{CpdConfiguration, CpdRegistration, CpdRegistrationState}
import apps.DemoDatasetLoader
import scalismo.geometry.{_3D, EuclideanVector, Point}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.PointDistributionModel
import scalismo.transformations.{Rotation, Translation, TranslationAfterRotation}
import scalismo.utils.Random.implicits.randomGenerator

import java.io.File

object DemoCPD extends App {
  scalismo.initialize()

  def run(
    model: PointDistributionModel[_3D, TriangleMesh],
    target: TriangleMesh[_3D],
    discretization: Int,
    maxIterations: Int,
    probabilistic: Boolean,
    initSigma: Option[Double],
    transform: GlobalTranformationType
  ): GeneralRegistrationState = {
    val configCPD = CpdConfiguration(maxIterations = maxIterations, threshold = 1e-10, lambda = 1.0, initialSigma = initSigma)
    val algorithmCPD = new CpdRegistration()

    val simpleRegistration = new SimpleRegistrator[CpdRegistrationState, CpdRegistration, CpdConfiguration](
      model,
      target,
      algorithm = algorithmCPD,
      config = configCPD,
      evaluatorUncertainty = 2.0,
      transform = transform,
      jsonFile = Some(new File("data/femur/targetFittingCPD.json"))
    )

    simpleRegistration.runDecimated(modelPoints = discretization, targetPoints = discretization, probabilistic = probabilistic, randomMixture = 0.50).general
  }

  val rigidOffset = TranslationAfterRotation(Translation(EuclideanVector(0, 0, 0)), Rotation(0.0, 0.0, 0.0, Point(0, 0, 0)))
  val (model, _) = DemoDatasetLoader.modelFemur()
  val (target, _) = DemoDatasetLoader.targetFemur(offset = rigidOffset)

  run(model, target, discretization = 100, maxIterations = 100, probabilistic = false, initSigma = None, RigidTransforms)
  run(model, target, discretization = 100, maxIterations = 10000, probabilistic = true, initSigma = Some(50), NoTransforms)
}
