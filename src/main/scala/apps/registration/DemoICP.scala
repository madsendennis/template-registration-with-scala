package apps.registration

import api.NoTransforms
import apps.DemoDatasetLoader
import scalismo.geometry.{EuclideanVector, Point}
import scalismo.transformations.{Rotation, Translation, TranslationAfterRotation}
import scalismo.utils.Random.implicits.randomGenerator

import java.io.File

object DemoICP extends App {
  scalismo.initialize()

  val rigidOffset = TranslationAfterRotation(Translation(EuclideanVector(0, 0, 0)), Rotation(0, 0, 0, Point(0, 0, 0)))
  val (model, _) = DemoDatasetLoader.femur.modelGauss(decimate = Some(1000), scaling = 1.0, sigma = 50)
  val (target, _) = DemoDatasetLoader.femur.target(offset = rigidOffset)

  // Run deterministic ICP
  val configDeterministic = new DemoConfigurations(model, target, discretization = 100, maxIterations = 100, probabilistic = false, transform = NoTransforms)
  configDeterministic.ICP()

  // Run probabilistic IPC
  val configProbabilistic = new DemoConfigurations(
    model,
    target,
    discretization = 100,
    maxIterations = 10000,
    probabilistic = true,
    transform = NoTransforms,
    jsonFile = Some(new File(DemoDatasetLoader.dataPath, "femur/targetFittingICP.json")))
  configProbabilistic.ICP()
}
