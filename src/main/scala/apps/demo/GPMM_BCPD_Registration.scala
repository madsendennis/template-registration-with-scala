package apps.demo

import java.awt.Color
import java.io.File

import api.registration.{GpmmBcpdRegistration, GpmmSpecialICPRegistration}
import api.registration.utils.{GPMMHelper, RigidTransforms, modelViewer}
import scalismo.geometry.{EuclideanVector3D, Point3D, _3D}
import scalismo.io.MeshIO
import scalismo.mesh.TriangleMesh
import scalismo.transformations.{Rotation3D, Scaling, Translation3D, TranslationAfterScalingAfterRotation}
import scalismo.ui.api.ScalismoUI

object GPMM_BCPD_Registration extends App {
  scalismo.initialize()

  val globalTrans = TranslationAfterScalingAfterRotation(Translation3D(EuclideanVector3D(20.0, 5.0, -5.0)), Scaling[_3D](1.0), Rotation3D(0.1, 0.0, 0.0, Point3D(0, 0, 0)))

  val template = MeshIO.readMesh(new File("data/femur_reference.stl")).get.operations.decimate(100)
  val target = MeshIO.readMesh(new File("data/femur_target.stl")).get.operations.decimate(100).transform(globalTrans)

  println(s"Template points: ${template.pointSet.numberOfPoints}, triangles: ${template.triangles.length}")
  println(s"Target points: ${target.pointSet.numberOfPoints}, triangles: ${target.triangles.length}")

  val gpmm = GPMMHelper.automaticGPMMfromTemplate(template, relativeTolerance = 0.01)

  println(s"Model rank: ${gpmm.rank} with ${template.pointSet.numberOfPoints} points")

  val ui = ScalismoUI("BCPD as GPMM-regression")
  val dataGroup = ui.createGroup("data")
  val modelGroup = ui.createGroup("model")
  val modelView = ui.show(modelGroup, gpmm, "model")
  ui.show(dataGroup, template, "template").color = Color.RED
  ui.show(dataGroup, target, "target").color = Color.GREEN

    val registration = new GpmmBcpdRegistration[_3D, TriangleMesh](gpmm,
      target,
      w = 0.0,
      lambda = 1,
      gamma = 1.0,
      k = 1000,
      max_iterations = 50,
      Option(modelViewer(modelView.shapeModelTransformationView, 5))
    )

//  val registration = new GpmmSpecialICPRegistration(gpmm,
//    target,
//    w = 0.0,
//    lambda = 1,
//    gamma = 1.0,
//    k = 1000,
//    max_iterations = 100,
//    Option(modelViewer(modelView.shapeModelTransformationView, 5))
//  )

  val t10 = System.currentTimeMillis()
  val (fitPars, fitTrans) = registration.register(tolerance = 0.000001, transformationType = RigidTransforms)
  val t11 = System.currentTimeMillis()
  println(s"Fitting time: ${(t11 - t10) / 1000.0} sec.")

  val myTrans = TranslationAfterScalingAfterRotation(fitTrans.t, fitTrans.s, fitTrans.R)
  val fit = gpmm.instance(fitPars).transform(myTrans)
  println(s"Fit points: ${fit.pointSet.numberOfPoints}, triangles: ${fit.triangles.length}")

  println(s"Final transformation, s: ${fitTrans.s.s}, t: ${fitTrans.t.t.toBreezeVector}, \n R: ${fitTrans.R.rotationMatrix}")

  ui.show(dataGroup, fit, "fit")
}
