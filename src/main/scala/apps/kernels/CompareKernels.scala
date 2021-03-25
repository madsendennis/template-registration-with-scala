package apps.kernels

import java.io.File

import api.registration.utils.{GPMMTriangleMesh3D, GaussianKernelParameters, PointSetHelper}
import scalismo.geometry.{EuclideanVector, Point, _3D}
import scalismo.io.{LandmarkIO, MeshIO, StatisticalModelIO}
import scalismo.mesh.TriangleMesh
import scalismo.transformations.{Rotation, Scaling, Translation, TranslationAfterRotation, TranslationAfterScalingAfterRotation}
import scalismo.ui.api.ScalismoUI
import scalismo.utils.Random.implicits.randomGenerator

object CompareKernels extends App{
  scalismo.initialize()

  val transform = TranslationAfterRotation[_3D](Translation(EuclideanVector(0, 0, 0)), Rotation(0, math.Pi, 0, Point(0,0,0)))
  val armadillo = MeshIO.readMesh(new File("data/armadillo/armadillo_scaled.ply")).get.operations.decimate(2000)
  val armailloLms = LandmarkIO.readLandmarksJson3D(new File("data/armadillo/armadillo.json")).get

  val toePoint = armailloLms.find(_.id=="foot_right").get
  val toeId = armadillo.pointSet.findClosestPoint(toePoint.point).id

  val reference = armadillo

  val ps = PointSetHelper[_3D, TriangleMesh](reference)
  val maxDist = ps.maximumPointDistance(reference.pointSet)
  val minDist = ps.minimumPointDistance(reference.pointSet)

  val gpmmHelp = GPMMTriangleMesh3D(reference, relativeTolerance = 0.0)
  println("Starting")
  val gpmmModelL = gpmmHelp.GaussianMixture(Seq(GaussianKernelParameters(maxDist/4, maxDist/8))).truncate(1000)
  StatisticalModelIO.writeStatisticalTriangleMeshModel3D(gpmmModelL, new File("data/armadillo/models/gaussL_high.h5"))
  println("Large computed")
//  val gpmmModelM = gpmmHelp.GaussianMixture(Seq(GaussianKernelParameters(maxDist/8, maxDist/16))).truncate(100)
//  StatisticalModelIO.writeStatisticalTriangleMeshModel3D(gpmmModelM, new File("data/armadillo/models/gaussM.h5"))
//  println("Medium computed")
//  val gpmmModelS = gpmmHelp.GaussianMixture(Seq(GaussianKernelParameters(maxDist/16, maxDist/32))).truncate(100)
//  StatisticalModelIO.writeStatisticalTriangleMeshModel3D(gpmmModelS, new File("data/armadillo/models/gaussS.h5"))
//  println("Small computed")
//  val gpmmModelMix = gpmmHelp.AutomaticGaussian().truncate(100)
//  StatisticalModelIO.writeStatisticalTriangleMeshModel3D(gpmmModelMix, new File("data/armadillo/models/gaussMix.h5"))
//  println("Mix computed")
  val gpmmModelInvLap = GPMMTriangleMesh3D(reference, relativeTolerance = 0.0).InverseLaplacian(50.0).truncate(1000)
  StatisticalModelIO.writeStatisticalTriangleMeshModel3D(gpmmModelInvLap, new File("data/armadillo/models/invLap_high.h5"))
  println("inv lap")
//  val gpmmModelInvLapDot = GPMMTriangleMesh3D(reference, relativeTolerance = 0.0).InverseLaplacianDot(0.05, 1.0).truncate(100)
//  StatisticalModelIO.writeStatisticalTriangleMeshModel3D(gpmmModelInvLapDot, new File("data/armadillo/models/invLapDot.h5"))
//  println("inv lap dot")
//  val gpmmModelLsymm = GPMMTriangleMesh3D(reference, relativeTolerance = 0.0).GaussianSymmetry(maxDist/4, maxDist/8).truncate(100)
//  StatisticalModelIO.writeStatisticalTriangleMeshModel3D(gpmmModelLsymm, new File("data/armadillo/models/gaussLsymm.h5"))
//  println("gauss mirror")
//  val path = new File("data/armadillo/models")
//  val gpmmModel = StatisticalModelIO.readStatisticalTriangleMeshModel3D(new File(path, "paper/mirror.h5")).get.transform(transform)
}
