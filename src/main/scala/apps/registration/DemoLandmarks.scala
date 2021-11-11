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
  val rigidOffset = TranslationAfterRotation(Translation(EuclideanVector(0, 0, 0)), Rotation(0, 0, 0, Point(0, 0, 0)))

  val (modelLowRes, modelLandmarks) = DemoDatasetLoader.modelArmadillo()
  val (target, targetLandmarks) = DemoDatasetLoader.targetArmadillo(offset = rigidOffset)
  val (ref, _) = DemoDatasetLoader.referenceArmadillo()

  val model = modelLowRes.newReference(ref.operations.decimate(10000), TriangleMeshInterpolator3D()).truncate(200)

  val configCPD = CpdConfiguration(maxIterations = 100, threshold = 1e-5, lambda = 1.0)
  val algorithmCPD = new CpdRegistration()

  val simpleRegistrationLandmarks = new SimpleRegistrator[CpdRegistrationState, CpdRegistration, CpdConfiguration](
    model,
    target,
    modelLandmarks = Some(modelLandmarks),
    targetLandmarks = Some(targetLandmarks),
    algorithm = algorithmCPD,
    config = configCPD,
    evaluatorUncertainty = 2.0
  )

  val finalCPD = simpleRegistrationLandmarks.runDecimated(modelPoints = 200, targetPoints = 200, probabilistic = false)

}
