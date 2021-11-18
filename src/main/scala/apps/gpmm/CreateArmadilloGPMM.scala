package apps.gpmm

import java.io.File

import api.gpmm.GPMMTriangleMesh3D
import apps.DemoDatasetLoader
import scalismo.io.StatisticalModelIO
import scalismo.ui.api.ScalismoUI

object CreateArmadilloGPMM extends App {
  scalismo.initialize()

  // Reduce decimation points for lower-appromation and faster computation
  //  val (model, _) = DemoDatasetLoader.armadillo.modelGauss(Some(100), scaling = 1.0, sigma = 50)
  val (model, _) = DemoDatasetLoader.armadillo.modelGauss(Some(10000))
  val ui = ScalismoUI()
  ui.show(model, "model")
}
