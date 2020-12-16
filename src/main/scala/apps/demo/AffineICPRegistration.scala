package apps.demo

import java.awt.Color
import java.io.File

import api.registration.RigidICPRegistration
import api.registration.utils.PoseRegistrator.RigidRegistrator3D
import scalismo.geometry.{EuclideanVector3D, Point3D, _3D}
import scalismo.io.MeshIO
import scalismo.mesh.TriangleMesh
import scalismo.transformations._
import scalismo.ui.api.ScalismoUI

object AffineICPRegistration extends App {
  scalismo.initialize()

  val affineTrans: SimilarityTransformation[_3D] = TranslationAfterScalingAfterRotation3D(Translation3D(EuclideanVector3D(50.0, 20.0, 30.0)), Scaling3D(2), Rotation3D(0.1, 0.0, 0.0, Point3D(0, 0, 0)))

  val template: TriangleMesh[_3D] = MeshIO.readMesh(new File("data/femur_reference.stl")).get
  val target: TriangleMesh[_3D] = MeshIO.readMesh(new File("data/femur_target.stl")).get.transform(affineTrans)

  val reg = new RigidICPRegistration(template, 10)
  val fit = reg.register(target)

  val ui = ScalismoUI()
  val dataGroup = ui.createGroup("data")
  ui.show(dataGroup, template, "template").color = Color.RED
  ui.show(dataGroup, target, "target").color = Color.GREEN
  ui.show(dataGroup, fit, "fit")
}
