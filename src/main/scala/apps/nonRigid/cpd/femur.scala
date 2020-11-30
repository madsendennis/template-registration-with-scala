package apps.NonRigid.cpd

import java.awt.Color
import java.io.File

import api.registration.CPDNonRigid
import scalismo.io.MeshIO
import scalismo.ui.api.ScalismoUI

object femur extends App {

  scalismo.initialize()

  // low resolution aligned femur meshes ~100 vertices
  val source = MeshIO.readMesh(new File("data/femur0_coarse.stl")).get
  val target = MeshIO.readMesh(new File("data/femur1_coarse.stl")).get

  // "high" resolution femur meshes ~1600 vertices
  //    val source = MeshIO.readMesh(new File("data/femur_reference.stl")).get
  //    val target = MeshIO.readMesh(new File("data/femur_target.stl")).get
  //    val target = MeshIO.readMesh(new File("data/femur_target_partial.stl")).get
  //    val target = MeshIO.readMesh(new File("data/femur_target_rigid.stl")).get
  //    val target = MeshIO.readMesh(new File("data/femur_target_rigid_rotate.stl")).get


  println(s"CPD Femur fun - Source points: ${source.pointSet.numberOfPoints}, target points: ${target.pointSet.numberOfPoints}")

  // lambda and beta both reflect the amount of smoothness regularization
  // beta = width of gaussian filter
  // lambda = trade-off between goodness of fit and regularization
  val cpd = new CPDNonRigid(target, source, lamdba = 10, beta = 100, w = 0.0) // target, source ... Source is moving!!!

  val finalMesh = cpd.Registration(max_iteration = 1000, tolerance = 0.001)
  val ui = ScalismoUI()
  val showTarget = ui.show(target, "target")
  val showSource = ui.show(source, "source")
  val showFinal = ui.show(finalMesh, "final")
  showTarget.color = Color.GREEN
  showSource.color = Color.RED
}
