package apps.demo

import java.awt.Color
import java.io.File

import api.registration.icp.NonRigidICPwithGPMMTriangle3D
import api.registration.utils.GPMMTriangleMesh3D
import scalismo.io.MeshIO
import scalismo.ui.api.ScalismoUI

object GPMM_ICP_Registration extends App {
  scalismo.initialize()

  val template = MeshIO.readMesh(new File("data/femur_reference.stl")).get
  val target = MeshIO.readMesh(new File("data/femur_target.stl")).get

  val gpmm = GPMMTriangleMesh3D(template, relativeTolerance = 0.1).AutomaticGaussian()

  println(s"Template points: ${template.pointSet.numberOfPoints}, triangles: ${template.triangles.length}")
  println(s"Target points: ${target.pointSet.numberOfPoints}, triangles: ${target.triangles.length}")

  val nicp = new NonRigidICPwithGPMMTriangle3D(gpmm, target, None) // Without landmarks

  val t10 = System.currentTimeMillis()
  val fitPars = nicp.Registration(10, 0.0000001)
  val t11 = System.currentTimeMillis()
  println(s"Fitting time: ${(t11 - t10) / 1000.0} sec.")
  val fit = gpmm.instance(fitPars)

  println(s"Fit points: ${fit.pointSet.numberOfPoints}, triangles: ${fit.triangles.length}")

  val ui = ScalismoUI()
  val dataGroup = ui.createGroup("data")
  ui.show(dataGroup, template, "template").color = Color.RED
  ui.show(dataGroup, target, "target").color = Color.GREEN
  ui.show(dataGroup, fit, "fit")
}
