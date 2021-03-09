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

  val (template, templateLms, target, targetLms) = {
    val template = MeshIO.readMesh(new File("data/femur_reference.stl")).get
    val target = MeshIO.readMesh(new File("data/femur_target.stl")).get
    val templateLms = LandmarkIO.readLandmarksJson3D(new File("data/femur_reference.json")).get
    val targetLms = LandmarkIO.readLandmarksJson3D(new File("data/femur_target.json")).get
//
//    val minX = Math.min(
//      template.pointSet.pointSequence.map(_.x).min,
//      target.pointSet.pointSequence.map(_.x).min
//    )
//    val minY = Math.min(
//      template.pointSet.pointSequence.map(_.y).min,
//      target.pointSet.pointSequence.map(_.y).min
//    )
//    val minZ = Math.min(
//      template.pointSet.pointSequence.map(_.z).min,
//      target.pointSet.pointSequence.map(_.z).min
//    )
//    val maxX = Math.max(
//      template.pointSet.pointSequence.map(_.x).max,
//      target.pointSet.pointSequence.map(_.x).max
//    )
//    val maxY = Math.max(
//      template.pointSet.pointSequence.map(_.y).max,
//      target.pointSet.pointSequence.map(_.y).max
//    )
//    val maxZ = Math.max(
//      template.pointSet.pointSequence.map(_.z).max,
//      target.pointSet.pointSequence.map(_.z).max
//    )
//
//    def to_11(x: Double, min: Double, max: Double): Double = {
//      (x - min) / (max - min) * 2.0 - 1.0
//    }
//
//    def scale(pt: Point[_3D]): Point[_3D] = {
//      Point(to_11(pt.x, minX, maxX), to_11(pt.y, minY, maxY), to_11(pt.z, minZ, maxZ))
//    }
//
//    (
//      template.transform(scale),
//      templateLms.map(lm => lm.copy(point = scale(lm.point))),
//      target.transform(scale),
//      targetLms.map(lm => lm.copy(point = scale(lm.point)))
//    )

    (template, templateLms, target, targetLms)
  }
  println(s"Template points: ${template.pointSet.numberOfPoints}, triangles: ${template.triangles.length}")
  println(s"Target points: ${target.pointSet.numberOfPoints}, triangles: ${target.triangles.length}")

  val ui = ScalismoUI()
  val dataGroup = ui.createGroup("data")
  ui.show(dataGroup, template, "template").color = Color.RED
  ui.show(dataGroup, target, "target").color = Color.GREEN

  val iterGroup = ui.createGroup("iterations")
  // Choose between ICP-A and ICP-T
  //val nicp = new NonRigidOptimalStepICP_T(template, target, Seq(), Seq())
  val nicp = new NonRigidOptimalStepICP_A(template, target, templateLms, targetLms)

  val t0 = System.currentTimeMillis()
  val fit = nicp.Registration(2, 0.0000001, callback = (t: TriangleMesh[_3D], d: Double, lm: IndexedSeq[Point[_3D]]) => {
    ui.show(iterGroup, t, "iter")
    ui.show(iterGroup, lm, "landmarks")
  })
  val t1 = System.currentTimeMillis()
  println(s"Fitting time: ${(t1 - t0) / 1000.0} sec.")

  println(s"Fit points: ${fit.pointSet.numberOfPoints}, triangles: ${fit.triangles.length}")

  ui.show(dataGroup, fit, "fit")
}
