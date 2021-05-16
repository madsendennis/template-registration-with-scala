package apps.kernels

import java.io.File

import api.registration.utils.GPMMTriangleMesh3D
import scalismo.geometry.{EuclideanVector, _3D}
import scalismo.io.{LandmarkIO, MeshIO, StatisticalModelIO}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.DiscreteLowRankGaussianProcess
import scalismo.ui.api.ScalismoUI

object kernelsCheck extends App {
  scalismo.initialize()

  val armadillo = MeshIO.readMesh(new File("data/armadillo/armadillo_scaled.ply")).get
  val armailloLms = LandmarkIO.readLandmarksJson3D(new File("data/armadillo/armadillo.json")).get
  val toePoint = armailloLms.find(_.id=="foot_right").get
  val toeId = armadillo.pointSet.findClosestPoint(toePoint.point).id

  val modelNames = Seq("gaussL", "gaussM", "gaussS", "invLap", "invLapDot", "gaussLsymm")

  val modelName = modelNames(0)
  modelNames.foreach { modelName =>
    println(s"Model: ${modelName}")
    val model = StatisticalModelIO.readStatisticalTriangleMeshModel3D(new File(s"data/armadillo/models/${modelName}.h5")).get
    val karatemodel = StatisticalModelIO.readStatisticalTriangleMeshModel3D(new File(s"data/armadillo/karatemodels/${modelName}.h5")).get

    println(s"Model points: ${model.reference.pointSet.numberOfPoints}, karate: ${karatemodel.reference.pointSet.numberOfPoints}")
    val ui = ScalismoUI(modelName)
    val gpmmHelp = GPMMTriangleMesh3D(model.reference, relativeTolerance = 0.0)
    val colorNormal = gpmmHelp.computeDistanceAbsMesh(model, toeId)
    val colorKarate = gpmmHelp.computeDistanceAbsMesh(karatemodel, toeId)
    ui.show(colorNormal, "normal")
    ui.show(colorKarate, "karate")
    MeshIO.writeScalarMeshField[Double](colorNormal, new File(s"data/armadillo/thesiscolor/${modelName}_normal.vtk"))
    MeshIO.writeScalarMeshField[Double](colorKarate, new File(s"data/armadillo/thesiscolor/${modelName}_karate.vtk"))
  }
}
