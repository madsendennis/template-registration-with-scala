package apps.registration

import api.{GeneralRegistrationState, NoTransforms}
import api.registration.config.{IcpConfiguration, IcpRegistration, IcpRegistrationState}
import apps.DemoDatasetLoader
import scalismo.geometry.{_3D, EuclideanVector, Point}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.PointDistributionModel
import scalismo.transformations.{Rotation, Translation, TranslationAfterRotation}
import scalismo.utils.Random.implicits.randomGenerator
import java.io.File

import scalismo.io.MeshIO

object DemoICP extends App {
  scalismo.initialize()

  def run(
    model: PointDistributionModel[_3D, TriangleMesh],
    target: TriangleMesh[_3D],
    discretization: Int,
    maxIterations: Int,
    probabilistic: Boolean
  ): GeneralRegistrationState = {
    val configICP = IcpConfiguration(maxIterations = maxIterations, initialSigma = 1.0, endSigma = 1.0, reverseCorrespondenceDirection = true)
    val algorithmICP = new IcpRegistration()

    val simpleRegistration = new SimpleRegistrator[IcpRegistrationState, IcpRegistration, IcpConfiguration](
      model,
      target,
      algorithm = algorithmICP,
      config = configICP,
      evaluatorUncertainty = 2.0,
      transform = NoTransforms,
      jsonFile = Some(new File("data/femur/targetFittingICP.json"))
    )

    simpleRegistration.runDecimated(modelPoints = discretization, targetPoints = discretization, probabilistic = probabilistic, randomMixture = 0.50).general
  }

  val rigidOffset = TranslationAfterRotation(Translation(EuclideanVector(0, 0, 0)), Rotation(0, 0, 0, Point(0, 0, 0)))
  val (model, _) = DemoDatasetLoader.modelFemur()
  val (target, _) = DemoDatasetLoader.targetFemur(offset = rigidOffset)

  run(model, target, discretization = 100, maxIterations = 100, probabilistic = false)
  run(model, target, discretization = 100, maxIterations = 10000, probabilistic = true)
}
