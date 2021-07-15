package apps.demo

import java.awt.Color
import java.io.File

import api.registration.icp.NonRigidICPwithGPMMTriangle3D
import api.registration.{GpmmBcpdRegistration, GpmmCpdRegistration}
import api.registration.utils.{GPMMTriangleMesh3D, NoTransforms, RigidTransforms, modelViewer}
import scalismo.geometry.{EuclideanVector, Point, _3D}
import scalismo.io.{LandmarkIO, MeshIO, StatisticalModelIO}
import scalismo.mesh.TriangleMesh
import scalismo.transformations.{Rotation, Scaling, Translation, TranslationAfterScalingAfterRotation}
import scalismo.ui.api.ScalismoUI

object Armadillo extends App{
  scalismo.initialize()

  val target = MeshIO.readMesh(new File("data/armadillo/armadillo_karate.ply")).get
  val gpmm = StatisticalModelIO.readStatisticalTriangleMeshModel3D(new File("data/armadillo/models/invLapDot.h5")).get.truncate(500) // This works well - also without Landmarks
//  val gpmm = StatisticalModelIO.readStatisticalTriangleMeshModel3D(new File("data/armadillo/models/gaussL.h5")).get.truncate(500)


  val modelLMs = LandmarkIO.readLandmarksJson3D(new File("data/armadillo/armadillo.json")).get
  val targetLMs = modelLMs.map{lm =>
    val id = gpmm.reference.pointSet.findClosestPoint(lm.point).id
    lm.copy(point = target.pointSet.point(id))
  }

  println(s"target points: ${target.pointSet.numberOfPoints}")
  println(s"gpmm points: ${gpmm.reference.pointSet.numberOfPoints}, rank: ${gpmm.rank}")

  val ui = ScalismoUI("GiNGR")
  val targetGroup = ui.createGroup("target")
  val modelGroup = ui.createGroup("model")
  val showTarget = ui.show(targetGroup, target, "target")
  showTarget.color = Color.RED
  showTarget.opacity = 0.5
  val showModel = ui.show(modelGroup, gpmm, "model")

  ui.show(targetGroup, targetLMs, "landmarks").map(_.color = Color.RED)
  ui.show(modelGroup, modelLMs, "landmarks")

  val mv = Option(modelViewer(showModel.shapeModelTransformationView, 1))
  val downSampleGPMM = gpmm //gpmm.newReference(gpmm.reference.operations.decimate(200), TriangleMeshInterpolator3D())
//  //  val gpmmRegister = new GpmmSpecialICPRegistration(gpmm = downSampleGPMM, targetPoints = target, w=0.0, lambda=1.0, gamma=1.0, k=1000, max_iterations = 100, modelView = mv)
////  val gpmmRegister = new GpmmBcpdRegistration[_3D, TriangleMesh](gpmm = downSampleGPMM, targetPoints = target, w=0.0, lambda=1.0, gamma=100.0, k=1000, max_iterations = 100, modelView = mv)
////  val fit = gpmmRegister.register(tolerance = 0.000001, transformationType = RigidTransforms)

//  val gpmmRegister = new GpmmCpdRegistration[_3D, TriangleMesh](gpmm, target, Seq(), Seq(), lambda = 1.0, w = 0.0, max_iterations = 100, modelView = mv)
//  val gpmmRegister = new GpmmCpdRegistration[_3D, TriangleMesh](gpmm, target, modelLMs, targetLMs, lambda = 1.0, w = 0.0, max_iterations = 100, modelView = mv)

//  println("Sleep 10 sec.")
//  Thread.sleep(10000)
//
//  val fit = gpmmRegister.registerAndWarp(tolerance = 0.00000001)

//  val gpmmRegister = new NonRigidICPwithGPMMTriangle3D(gpmm, target, modelView = mv)
//  val fit = gpmmRegister.Registration(tolerance = 0.000000001, max_iteration = 10, sigma2 = Seq(1000.0, 100.0, 10.0, 1.0, 0.1))
}
