package apps

import java.io.File

import api.registration.config.{CpdConfiguration, CpdRegistration, CpdRegistrationState}
import scalismo.geometry._3D
import scalismo.io.{MeshIO, StatisticalModelIO}
import scalismo.mesh.TriangleMesh
import scalismo.utils.Random.implicits.randomGenerator

object SimpleDemo extends App {
  scalismo.initialize()

  val model = StatisticalModelIO.readStatisticalTriangleMeshModel3D(new File("data/femur_gp_model_50-components.h5")).get
  val target: TriangleMesh[_3D] = MeshIO.readMesh(new File("data/femur_target.stl")).get

  val configCPD = CpdConfiguration(maxIterations = 100, threshold = 1e-20)
  val algorithmCPD = new CpdRegistration()

  val simpleRegistration = new SimpleRegistrator[CpdRegistrationState, CpdRegistration, CpdConfiguration](
    model,
    target,
    algorithmCPD,
    configCPD
  )
  val finalCPD = simpleRegistration.runDecimated(modelPoints = 200, targetPoints = 200, probabilistic = false)
}
