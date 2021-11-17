package apps.gpmm

import java.io.File

import api.gpmm.GPMMTriangleMesh3D
import apps.DemoDatasetLoader
import scalismo.io.StatisticalModelIO
import scalismo.ui.api.ScalismoUI

object CreateArmadilloGPMM extends App {
  scalismo.initialize()

  val (ref, _) = DemoDatasetLoader.armadillo.model(Some(1000), WhichKernel)
  // Need to decimate as inverse laplacian based kernels spans the full covariance matrix
  val decimated = ref.operations.decimate(1000)
  println("Decimated")
  val gpmmHelper = GPMMTriangleMesh3D(decimated, relativeTolerance = 0.01)
  val model = gpmmHelper.InverseLaplacianDot(0.05, 1.0)

  StatisticalModelIO.writeStatisticalTriangleMeshModel3D(model.truncate(500), new File(DemoDatasetLoader.pathArmadillo, "armadillo-invlapdot-1000.h5"))

  val ui = ScalismoUI()
  ui.show(model, "model")
}
