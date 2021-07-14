package apps.demo

import java.awt.Color
import java.io.File

import api.registration.icp.{NonRigidOptimalStepICP_A, NonRigidOptimalStepICP_T}
import scalismo.geometry.{Point, _3D}
import scalismo.io.{LandmarkIO, MeshIO}
import scalismo.mesh.TriangleMesh
import scalismo.ui.api.ScalismoUI

object NonRigidOptimalStepICPRegistration extends App {
  scalismo.initialize()

  val template = MeshIO.readMesh(new File("data/femur_reference.stl")).get.operations.decimate(200)
  val target = MeshIO.readMesh(new File("data/femur_target.stl")).get
  val templateLms = LandmarkIO.readLandmarksJson3D(new File("data/femur_reference.json")).get
  val targetLms = LandmarkIO.readLandmarksJson3D(new File("data/femur_target.json")).get

  println(s"Template points: ${template.pointSet.numberOfPoints}, triangles: ${template.triangles.length}")
  println(s"Target points: ${target.pointSet.numberOfPoints}, triangles: ${target.triangles.length}")

  val ui = ScalismoUI()
  val dataGroup = ui.createGroup("data")
  ui.show(dataGroup, template, "template").color = Color.RED
  ui.show(dataGroup, target, "target").color = Color.GREEN

  val iterGroup = ui.createGroup("iterations")
  // Choose between ICP-A and ICP-T
  val nicp = new NonRigidOptimalStepICP_T(template, target, templateLms, targetLms)
//  val nicp = new NonRigidOptimalStepICP_A(template, target, templateLms, targetLms)

  val t0 = System.currentTimeMillis()
  val fit = nicp.Registration(10, 0.0000001)._1
  val t1 = System.currentTimeMillis()
  println(s"Fitting time: ${(t1 - t0) / 1000.0} sec.")

  println(s"Fit points: ${fit.pointSet.numberOfPoints}, triangles: ${fit.triangles.length}")

  ui.show(dataGroup, fit, "fit")
}
