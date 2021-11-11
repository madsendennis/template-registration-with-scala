package apps.gpmm

import java.io.File

import api.gpmm.GPMMTriangleMesh3D
import apps.DemoDatasetLoader
import scalismo.io.StatisticalModelIO
import scalismo.ui.api.ScalismoUI

object CreateFemurGPMM extends App {
  scalismo.initialize()

  val (ref, _) = DemoDatasetLoader.referenceFemur()
  println("Decimated")
  val gpmmHelper = GPMMTriangleMesh3D(ref, relativeTolerance = 0.01)
  val model = gpmmHelper.AutomaticGaussian()

  StatisticalModelIO.writeStatisticalTriangleMeshModel3D(model, new File(DemoDatasetLoader.pathArmadillo, "femur.h5"))

  val ui = ScalismoUI()
  ui.show(model, "model")
}
