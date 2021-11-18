package apps.registration

import api.registration.config.{CpdConfiguration, CpdRegistration, CpdRegistrationState}
import apps.DemoDatasetLoader
import scalismo.common.interpolation.{NearestNeighborInterpolator, TriangleMeshInterpolator3D}
import scalismo.geometry.{EuclideanVector, Point}
import scalismo.transformations.{Rotation, Translation, TranslationAfterRotation}
import scalismo.ui.api.ScalismoUI
import scalismo.utils.Random.implicits.randomGenerator

object DemoLandmarks extends App {
  scalismo.initialize()
  val (model, modelLandmarks) = DemoDatasetLoader.armadillo.modelGauss(Some(100))
  val (target, targetLandmarks) = DemoDatasetLoader.armadillo.target()

  val configCPD = CpdConfiguration(maxIterations = 100, threshold = 1e-5, lambda = 1.0)
  val algorithmCPD = new CpdRegistration()

  val simpleRegistrationLandmarks = new SimpleRegistrator[CpdRegistrationState, CpdRegistration, CpdConfiguration](
    model,
    target,
    modelLandmarks = modelLandmarks,
    targetLandmarks = targetLandmarks,
    algorithm = algorithmCPD,
    config = configCPD,
    evaluatorUncertainty = 2.0
  )

  val finalCPD = simpleRegistrationLandmarks.runDecimated(modelPoints = 200, targetPoints = 200, probabilistic = false)

}
