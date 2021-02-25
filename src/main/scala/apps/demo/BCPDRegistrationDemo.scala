package apps.demo

import java.awt.Color
import java.io.File

import api.registration.{BCPDRegistration, NonRigidCPDRegistration}
import scalismo.geometry.{EuclideanVector3D, Point3D, _3D}
import scalismo.io.MeshIO
import scalismo.kernels.GaussianKernel
import scalismo.mesh.TriangleMesh
import scalismo.transformations.{RigidTransformation, Rotation3D, Scaling, Translation3D, TranslationAfterRotation3D, TranslationAfterScalingAfterRotation}
import scalismo.ui.api.ScalismoUI

object BCPDRegistrationDemo extends App {
  scalismo.initialize()

  val globalTrans = TranslationAfterScalingAfterRotation(Translation3D(EuclideanVector3D(20.0, 5.0, 5.0)), Scaling[_3D](1.5), Rotation3D(0.4, 0.0, 0.0, Point3D(0, 0, 0)))

  val template = MeshIO.readMesh(new File("data/femur_reference.stl")).get.operations.decimate(100)
  val target = MeshIO.readMesh(new File("data/femur_target.stl")).get.operations.decimate(100).transform(globalTrans)

  println(s"Template points: ${template.pointSet.numberOfPoints}, triangles: ${template.triangles.length}")
  println(s"Target points: ${target.pointSet.numberOfPoints}, triangles: ${target.triangles.length}")

  val myKernel = GaussianKernel[_3D](50)
  val bcpd = new BCPDRegistration(template, target, w = 0.0, lambda=1.0, gamma = 1.0, k=1.0, kernel = myKernel, max_iterations = 50)

  val t0 = System.currentTimeMillis()
  val fit = bcpd.register()
  val t1 = System.currentTimeMillis()
  println(s"Fitting time: ${(t1 - t0) / 1000.0} sec.")

  println(s"Fit points: ${fit.pointSet.numberOfPoints}, triangles: ${fit.triangles.length}")

  val ui = ScalismoUI("BCPD")
  val dataGroup = ui.createGroup("data")
  ui.show(dataGroup, template, "template").color = Color.RED
  ui.show(dataGroup, target, "target").color = Color.GREEN
  ui.show(dataGroup, fit, "fit")
}
