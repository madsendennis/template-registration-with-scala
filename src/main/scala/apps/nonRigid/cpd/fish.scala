package apps.NonRigid.cpd

import java.awt.Color
import java.io.File

import api.registration.CPDNonRigid
import scalismo.common._
import scalismo.io.MeshIO
import scalismo.ui.api.ScalismoUI

object fish extends App {

  scalismo.initialize()

  // Convert "trianglemeshes" to point cloud domain - otherwise nothing shows up in the UI as no triangles are defined.
  val target = MeshIO.readMesh(new File("data/fish0.ply")).get
  val targetPoints = UnstructuredPointsDomain.Create.CreateUnstructuredPointsDomain3D.create(target.pointSet.points.toIndexedSeq)
  val source = MeshIO.readMesh(new File("data/fish1.ply")).get
  val sourcePoints = UnstructuredPointsDomain.Create.CreateUnstructuredPointsDomain3D.create(source.pointSet.points.toIndexedSeq)
  val cpd = new CPDNonRigid(targetPoints, sourcePoints, lamdba = 2, beta = 2, w = 0.0) // target, source ... Source is moving!!!
  val finalReg = cpd.Registration(max_iteration = 20)

  val ui = ScalismoUI()
  val showTarget = ui.show(targetPoints, "target")
  val showSource = ui.show(sourcePoints, "source")
  val showFinal = ui.show(finalReg, "final")
  showTarget.radius = 0.1
  showSource.radius = 0.1
  showFinal.radius = 0.1
  showTarget.color = Color.GREEN
  showSource.color = Color.RED
}
