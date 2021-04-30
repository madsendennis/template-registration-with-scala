package apps.registration

import java.awt.Color
import java.io.File

import api.registration.icp.NonRigidICPwithGPMMTriangle3D
import api.registration.{GpmmBcpdRegistration, GpmmSpecialICPRegistration}
import api.registration.utils.{GPMMTriangleMesh3D, RigidTransforms, modelViewer}
import scalismo.common.interpolation.TriangleMeshInterpolator3D
import scalismo.geometry.{EuclideanVector, Point, _3D}
import scalismo.io.{MeshIO, StatisticalModelIO}
import scalismo.mesh.TriangleMesh
import scalismo.transformations.{Rotation, Scaling, Translation, TranslationAfterRotation, TranslationAfterScalingAfterRotation}
import scalismo.ui.api.ScalismoUI

object Femur extends App{
  scalismo.initialize()

  val reference = MeshIO.readMesh(new File("data/femur_reference.stl")).get
  val target = MeshIO.readMesh(new File("data/femur_target_partial.stl")).get
  val gt = MeshIO.readMesh(new File("data/femur_target.stl")).get


  val truncate = 400
  val gpmmHelp = GPMMTriangleMesh3D(reference, relativeTolerance = 0.0)
//  val gpmmGauss = gpmmHelp.AutomaticGaussian().truncate(truncate)
//  val gpmmInvLap = gpmmHelp.InverseLaplacian(50).truncate(truncate)
//  val gpmmInvLapDot = gpmmHelp.InverseLaplacianDot(0.05, 1.0).truncate(truncate)

  val gpmm = StatisticalModelIO.readStatisticalTriangleMeshModel3D(new File("data/femur_gauss_mix.h5")).get.truncate(truncate)
//  val gpmm = StatisticalModelIO.readStatisticalTriangleMeshModel3D(new File("data/femur_invLap.h5")).get.truncate(truncate)
//  val gpmm = StatisticalModelIO.readStatisticalTriangleMeshModel3D(new File("data/femur_invLapDot.h5")).get.truncate(truncate)

//  StatisticalModelIO.writeStatisticalTriangleMeshModel3D(gpmmGauss, new File("data/femur_gauss_mix.h5"))
//  StatisticalModelIO.writeStatisticalTriangleMeshModel3D(gpmmInvLap, new File("data/femur_invLap.h5"))
//  StatisticalModelIO.writeStatisticalTriangleMeshModel3D(gpmmInvLapDot, new File("data/femur_invLapDot.h5"))


    val ui = ScalismoUI()
  val targetGroup = ui.createGroup("target")
    val modelGroup = ui.createGroup("model")
  val showTarget = ui.show(targetGroup, target, "target")
  ui.show(targetGroup, gpmm.reference, "reference-init").opacity = 0.5
//  val showGT = ui.show(targetGroup, gt, "grount-truth")
//  showGT.color = Color.ORANGE
//  showGT.opacity = 0.0
  showTarget.color = Color.RED
  showTarget.opacity = 0.6

  val showModel = ui.show(modelGroup, gpmm, "model")
  val mv = Option(modelViewer(showModel.shapeModelTransformationView, 1))

//  val gpmmRegister = new GpmmSpecialICPRegistration(gpmm = downSampleGPMM, targetPoints = target, w=0.0, lambda=1.0, gamma=1.0, k=1000, max_iterations = 100, modelView = mv)
  println("Sleeping")
  Thread.sleep(50000)
  println("GOGOGO")
  val downSampleGPMM = gpmm.newReference(gpmm.reference.operations.decimate(200), TriangleMeshInterpolator3D())
  val gpmmRegister = new NonRigidICPwithGPMMTriangle3D(gpmm = downSampleGPMM, target.operations.decimate(400), modelView = mv)
  val fit = gpmmRegister.Registration(tolerance = 0.000000001, max_iteration = 20, sigma2 = Seq(10.0, 1.0, 0.1, 0.05))
}
