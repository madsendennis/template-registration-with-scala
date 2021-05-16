package apps.femurCourse

import java.io.File

import api.registration.GpmmCpdRegistration
import api.registration.utils.modelViewer
import apps.util.RegistrationComparison
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.io.{MeshIO, StatisticalModelIO}
import scalismo.ui.api.ScalismoUI

object step3_validateRegistrations extends App{
  scalismo.initialize()

  val meshesRegistered = data.registeredMeshes.listFiles(_.getName.endsWith(".stl")).sorted

  val meshesRaw = data.alignedMeshes.listFiles(_.getName.endsWith(".stl"))

  val tot = meshesRegistered.map{regFile =>
    val rawFile = new File(data.alignedMeshes, regFile.getName)

    val registered = MeshIO.readMesh(regFile).get
    val aligned = MeshIO.readMesh(rawFile).get

    RegistrationComparison.evaluateReconstruction2GroundTruthDouble(regFile.getName, registered, aligned)
  }
  println(s"Total, len: ${tot.length.toDouble}, avg: ${tot.map(_._1).sum/tot.length.toDouble}, hauss: ${tot.map(_._2).sum/tot.length.toDouble}")
}
