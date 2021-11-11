package apps

import api.registration.config.{IcpConfiguration, IcpRegistration, IcpRegistrationState}
import scalismo.geometry.{EuclideanVector, Point}
import scalismo.transformations.{Rotation, Translation, TranslationAfterRotation}
import scalismo.utils.Random.implicits.randomGenerator

object DemoICP extends App {
  scalismo.initialize()

  val rigidOffset = TranslationAfterRotation(Translation(EuclideanVector(0, 0, 0)), Rotation(0, 0, 0, Point(0, 0, 0)))
  val (model, _) = DemoDatasetLoader.modelFemur()
  val (target, _) = DemoDatasetLoader.targetFemur(offset = rigidOffset)

  val configICP = IcpConfiguration(maxIterations = 1000, threshold = 1e-10, initialSigma = 2.0, endSigma = 2.0)
  val algorithmICP = new IcpRegistration()

  val simpleRegistration = new SimpleRegistrator[IcpRegistrationState, IcpRegistration, IcpConfiguration](
    model,
    target,
    algorithmICP,
    configICP,
    evaluatorUncertainty = 1.0
  )

  val finalCPD = simpleRegistration.runDecimated(modelPoints = 100, targetPoints = 100, probabilistic = true)
}
