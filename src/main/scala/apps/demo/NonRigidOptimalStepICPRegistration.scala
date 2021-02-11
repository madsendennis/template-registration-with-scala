package apps.demo

import java.awt.Color
import java.io.File

import api.registration.icp.{NonRigidOptimalStepICP_A, NonRigidOptimalStepICP_T}
import scalismo.io.{LandmarkIO, MeshIO}
import scalismo.ui.api.ScalismoUI

object NonRigidOptimalStepICPRegistration extends App {
  scalismo.initialize()

  val template = MeshIO.readMesh(new File("data/femur_reference.stl")).get
  val target = MeshIO.readMesh(new File("data/femur_target.stl")).get
  val templateLms = LandmarkIO.readLandmarksJson3D(new File("data/femur_reference.json")).get
  val targetLms = LandmarkIO.readLandmarksJson3D(new File("data/femur_target.json")).get

  println(s"Template points: ${template.pointSet.numberOfPoints}, triangles: ${template.triangles.length}")
  println(s"Target points: ${target.pointSet.numberOfPoints}, triangles: ${target.triangles.length}")

  val t00 = System.currentTimeMillis()

  // Choose between ICP-A and ICP-T
//    val nicp = new NonRigidOptimalStepICP_T(template, target, Seq(), Seq())
  val nicp = new NonRigidOptimalStepICP_A(template, target, Seq(), Seq())

  val t01 = System.currentTimeMillis()
  println(s"Config time: ${(t01 - t00) / 1000} sec.")

  val t10 = System.currentTimeMillis()
  val fit = nicp.Registration(2, 0.0000001)
  val t11 = System.currentTimeMillis()
  println(s"Fitting time: ${(t11 - t10) / 1000.0} sec.")

  println(s"Fit points: ${fit.pointSet.numberOfPoints}, triangles: ${fit.triangles.length}")

  val ui = ScalismoUI()
  val dataGroup = ui.createGroup("data")
  ui.show(dataGroup, template, "template").color = Color.RED
  ui.show(dataGroup, target, "target").color = Color.GREEN
  ui.show(dataGroup, fit, "fit")
}
