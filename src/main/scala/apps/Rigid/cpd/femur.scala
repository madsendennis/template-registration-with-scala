package apps.Rigid.cpd

import java.awt.Color
import java.io.File

import api.registration.{RigidCPDRegistration}
import scalismo.geometry.{EuclideanVector3D, Point3D, _3D}
import scalismo.io.MeshIO
import scalismo.transformations.{RigidTransformation, Rotation3D, Translation3D, TranslationAfterRotation3D}
import scalismo.ui.api.ScalismoUI

object femur extends App {

  scalismo.initialize()

  val rigidTrans: RigidTransformation[_3D] =
    TranslationAfterRotation3D(Translation3D(EuclideanVector3D(50.0, 20.0, 30.0)), Rotation3D(0.1, 0.2, 0.3, Point3D(0, 0, 0)))

  val source = MeshIO.readMesh(new File("data/femur0_coarse.stl")).get.transform(rigidTrans)
  val target = MeshIO.readMesh(new File("data/femur1_coarse.stl")).get

  println(s"CPD Femur fun - Source points: ${source.pointSet.numberOfPoints}, target points: ${target.pointSet.numberOfPoints}")

  // lambda and beta both reflect the amount of smoothness regularization
  // beta = width of gaussian filter
  // lambda = trade-off between goodness of fit and regularization
  val cpd = new RigidCPDRegistration(source, lambda = 10, beta = 100, w = 0.0) // target, source ... Source is moving!!!

  val finalMesh = cpd.register(target)
  val ui = ScalismoUI()
  val showTarget = ui.show(target, "target")
  val showSource = ui.show(source, "source")
  val showFinal = ui.show(finalMesh, "final")
  showTarget.color = Color.GREEN
  showSource.color = Color.RED
}
