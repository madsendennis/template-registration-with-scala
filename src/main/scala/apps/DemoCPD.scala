package apps

import api.registration.config.{CpdConfiguration, CpdRegistration, CpdRegistrationState}
import scalismo.geometry.{EuclideanVector, Point}
import scalismo.transformations.{Rotation, Translation, TranslationAfterRotation}
import scalismo.utils.Random.implicits.randomGenerator

object DemoCPD extends App {
  scalismo.initialize()

  val rigidOffset = TranslationAfterRotation(Translation(EuclideanVector(0, 0, 0)), Rotation(0, 0, 0, Point(0, 0, 0)))
  val (model, _) = DemoDatasetLoader.modelFemur()
  val (target, _) = DemoDatasetLoader.targetFemur(offset = rigidOffset)

  val configCPD = CpdConfiguration(maxIterations = 10000, threshold = 1e-10, lambda = 1.0)
  val algorithmCPD = new CpdRegistration()

  val simpleRegistration = new SimpleRegistrator[CpdRegistrationState, CpdRegistration, CpdConfiguration](
    model,
    target,
    algorithmCPD,
    configCPD,
    evaluatorUncertainty = 2.0
  )

  val finalCPD = simpleRegistration.runDecimated(modelPoints = 100, targetPoints = 100, probabilistic = true)
}
