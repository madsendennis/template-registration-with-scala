package apps.registration

import api.{NoTransforms, RigidTransforms}
import api.registration.config.{CpdConfiguration, CpdRegistration, CpdRegistrationState}
import apps.DemoDatasetLoader
import scalismo.geometry.{EuclideanVector, Point}
import scalismo.transformations.{Rotation, Translation, TranslationAfterRotation}
import scalismo.utils.Random.implicits.randomGenerator

object DemoCPD extends App {
  scalismo.initialize()

  val rigidOffset = TranslationAfterRotation(Translation(EuclideanVector(0, 0, 0)), Rotation(0, 0, 0, Point(0, 0, 0)))
  val (model, _) = DemoDatasetLoader.modelFemur()
  val (target, _) = DemoDatasetLoader.targetFemur(offset = rigidOffset)

  val configCPD = CpdConfiguration(maxIterations = 10000, threshold = 1e-10, lambda = 1.0, initialSigma = Some(50.0))
  val algorithmCPD = new CpdRegistration()

  val simpleRegistration = new SimpleRegistrator[CpdRegistrationState, CpdRegistration, CpdConfiguration](
    model,
    target,
    algorithm = algorithmCPD,
    config = configCPD,
    evaluatorUncertainty = 2.0,
//    transform = RigidTransforms,
    transform = NoTransforms
  )

  val finalCPD = simpleRegistration.runDecimated(modelPoints = 100, targetPoints = 100, probabilistic = true, randomMixture = 0.50)
}
// Stochastic     ID:  average2surface: 1.1783928787436773 max: 4.975714076315487
// Deterministic: ID:  average2surface: 1.2968008309810397 max: 6.836591828165451
