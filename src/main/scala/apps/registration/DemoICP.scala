package apps.registration

import api.NoTransforms
import api.registration.config.{IcpConfiguration, IcpRegistration, IcpRegistrationState}
import apps.DemoDatasetLoader
import scalismo.geometry.{EuclideanVector, Point}
import scalismo.transformations.{Rotation, Translation, TranslationAfterRotation}
import scalismo.utils.Random.implicits.randomGenerator

object DemoICP extends App {
  scalismo.initialize()

  val rigidOffset = TranslationAfterRotation(Translation(EuclideanVector(0, 0, 0)), Rotation(0, 0, 0, Point(0, 0, 0)))
  val (model, _) = DemoDatasetLoader.modelFemur()
  val (target, _) = DemoDatasetLoader.targetFemur(offset = rigidOffset)

  val configICP = IcpConfiguration(maxIterations = 10000, threshold = 1e-10, initialSigma = 1.0, endSigma = 1.0)
  val algorithmICP = new IcpRegistration()

  val simpleRegistration = new SimpleRegistrator[IcpRegistrationState, IcpRegistration, IcpConfiguration](
    model,
    target,
    algorithm = algorithmICP,
    config = configICP,
    evaluatorUncertainty = 2.0,
    transform = NoTransforms
  )

  val finalCPD = simpleRegistration.runDecimated(modelPoints = 100, targetPoints = 100, probabilistic = true, randomMixture = 0.50)

  // ICP femur experiment: (400 points, 10.000 steps)
  // stochastic 0.50 (uncert: 2.0) (acc: 0.303653): ID:  average2surface: 1.1442718838808754 max: 4.414136071616627
  // stochastic 0.50 (uncert: 3.0) (acc: 0.445279): ID:  average2surface: 1.1894257780009618 max: 5.404303227472908
  // stochastic 0.50 (uncert: 5.0) (acc: 0.526676): ID:  average2surface: 1.203599395857474 max: 4.867729425860773
  // deterministic (100 steps):  ID:  average2surface: 1.1561254853388743 max: 4.853086040486575
  // deterministic (1000 steps): ID:  average2surface: 1.1139314921423664 max: 4.811508285951374

}
