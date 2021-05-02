package apps.registration

import java.awt.Color
import java.io.File

import api.registration.icp.NonRigidICPwithGPMMTriangle3D
import api.registration.{GpmmBcpdRegistration, GpmmCpdRegistration}
import api.registration.utils.{GPMMTriangleMesh3D, RigidTransforms, modelViewer}
import scalismo.common.interpolation.TriangleMeshInterpolator3D
import scalismo.geometry.{EuclideanVector, Point, _3D}
import scalismo.io.{MeshIO, StatisticalModelIO}
import scalismo.mesh.TriangleMesh
import scalismo.transformations.{Rotation, Scaling, Translation, TranslationAfterRotation, TranslationAfterScalingAfterRotation}
import scalismo.ui.api.{ScalismoUI, _3DLeft, _3DRight}
import scalismo.ui.view.properties.RigidTransformationPropertyPanel

object Bunny extends App{
  scalismo.initialize()

  val offSet = TranslationAfterScalingAfterRotation[_3D](Translation(EuclideanVector(30,30,0)), Scaling(0.9), Rotation(0,math.Pi/4,0, Point(0,0,0)))

  val referenceFull = MeshIO.readMesh(new File("data/bunny/bunny_reference.ply")).get
  val reference = referenceFull.operations.decimate(30000)
  val target = MeshIO.readMesh(new File("data/bunny/bunny_target.ply")).get.transform(offSet)

//  println(s"Reference points: ${referenceFull.pointSet.numberOfPoints}, low-ress: ${reference.pointSet.numberOfPoints}")

  val truncate = 500
//  val gpmmHelp = GPMMTriangleMesh3D(reference, relativeTolerance = 0.1)
//  val gpmmInit = gpmmHelp.AutomaticGaussian()
//  println(s"GPMM created with rank: ${gpmmInit.rank}")
//  val gpmm = gpmmInit.truncate(1000)
//  StatisticalModelIO.writeStatisticalTriangleMeshModel3D(gpmmInit, new File("data/bunny/bunny_gaussMix_30K.h5"))

  val gpmmInit = StatisticalModelIO.readStatisticalTriangleMeshModel3D(new File("data/bunny/bunny_gaussMix.h5")).get
  val gpmm = gpmmInit.truncate(truncate)
  println(s"GPMM with rank: ${gpmmInit.rank} and reference: ${gpmm.reference.pointSet.numberOfPoints}")

  val ui = ScalismoUI("GiNGR - Multi-resolution fitting")
  val targetGroup = ui.createGroup("target")
  val modelGroup = ui.createGroup("model")
  ui.show(targetGroup, target, "target").color = Color.RED
  val showModel = ui.show(modelGroup, gpmm, "model")
//  ui.show(modelGroup, referenceFull, "high-res")
  val mv = Option(modelViewer(showModel.shapeModelTransformationView, 1))

  ui.setVisibility(showModel.referenceView, List(_3DLeft))

  val downSampleGPMM1 = gpmm.newReference(gpmm.reference.operations.decimate(200), TriangleMeshInterpolator3D())

  val showLowRes = ui.show(modelGroup, downSampleGPMM1.reference, "low-res")
  ui.setVisibility(showLowRes, List(_3DRight))

  println("Sleeping")
  Thread.sleep(10000)
  println("Sleeping done")

  val gpmmRegister1 = new GpmmBcpdRegistration[_3D, TriangleMesh](gpmm = downSampleGPMM1, targetPoints = target.operations.decimate(500), w=0.0, lambda=1.0, gamma=1000.0, k=1, max_iterations = 50, modelView = mv)
  val fit1 = gpmmRegister1.register(tolerance = 0.001, transformationType = RigidTransforms)

  val downSampleGPMM2 = gpmm.newReference(gpmm.reference.operations.decimate(1000), TriangleMeshInterpolator3D()).transform(fit1._2.rigidTransform)

  val showMedRes = ui.show(modelGroup, downSampleGPMM2.reference.transform(fit1._2.rigidTransform.inverse), "medium-res")
  ui.setVisibility(showMedRes, List(_3DRight))
  ui.setVisibility(showLowRes, List())

  val gpmmRegister2 = new GpmmCpdRegistration[_3D, TriangleMesh](gpmm = downSampleGPMM2, target.operations.decimate(2000), Seq(), Seq(), w=0.0, lambda=1.0, max_iterations = 50, modelView = mv)
  val fit2 = gpmmRegister2.register(tolerance = 0.000001, initialGPMM = fit1._1, initialSigma2 = fit1._3)

  val gpmmRegister3 = new NonRigidICPwithGPMMTriangle3D(gpmm = gpmm.transform(fit1._2.rigidTransform), target.operations.decimate(10000), modelView = mv)
  val fit3 = gpmmRegister3.Registration(tolerance = 0.00000001, max_iteration = 5, sigma2 = Seq(0.1, 0.01, 0.001, 0.000001), initialGPMM = fit2)
}
