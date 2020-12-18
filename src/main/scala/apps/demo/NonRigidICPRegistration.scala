package apps.demo

import java.awt.Color
import java.io.File

import api.registration.icp.NonRigidICPwithGPMMTriangle3D
import api.registration.utils.GPMMHelper
import scalismo.io.MeshIO
import scalismo.ui.api.ScalismoUI

object NonRigidICPRegistration extends App {
  scalismo.initialize()

  val template = MeshIO.readMesh(new File("data/femur_reference.stl")).get
  val target = MeshIO.readMesh(new File("data/femur_target.stl")).get

  val gpmm = GPMMHelper.automaticGPMMfromTemplate(template)


  println(s"Template points: ${template.pointSet.numberOfPoints}, triangles: ${template.triangles.length}")
  println(s"Target points: ${target.pointSet.numberOfPoints}, triangles: ${target.triangles.length}")

  val nicp = new NonRigidICPwithGPMMTriangle3D(gpmm, target) // Without landmarks

  val t10 = System.currentTimeMillis()
  val fit = nicp.Registration(10, 0.0000001)
  val t11 = System.currentTimeMillis()
  println(s"Fitting time: ${(t11 - t10) / 1000.0} sec.")

  println(s"Fit points: ${fit.pointSet.numberOfPoints}, triangles: ${fit.triangles.length}")

  val ui = ScalismoUI()
  val dataGroup = ui.createGroup("data")
  ui.show(dataGroup, template, "template").color = Color.RED
  ui.show(dataGroup, target, "target").color = Color.GREEN
  ui.show(dataGroup, fit, "fit")
}
