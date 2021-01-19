package apps.demo

import java.awt.Color
import java.io.File

import api.registration.{GPMMRegistration, NonRigidCPDRegistration}
import scalismo.common.interpolation.{NearestNeighborInterpolator, TriangleMeshInterpolator3D}
import scalismo.common.{DomainWarp, EuclideanSpace, Field, RealSpace, UnstructuredPointsDomain}
import scalismo.geometry.{EuclideanVector, EuclideanVector3D, Point, _3D}
import scalismo.io.{MeshIO, StatisticalModelIO}
import scalismo.kernels.{DiagonalKernel, GaussianKernel, PDKernel}
import scalismo.mesh.{TriangleMesh, TriangleMesh3D}
import scalismo.statisticalmodel.{GaussianProcess, LowRankGaussianProcess, PointDistributionModel, StatisticalMeshModel}
import scalismo.ui.api.ScalismoUI

object NonRigidCPDwithGPMMRegistrationDemo extends App {
  scalismo.initialize()

  val template = MeshIO.readMesh(new File("data/femur_reference.stl")).get
//  val tv = EuclideanVector3D(0,0,0)
  val target = MeshIO.readMesh(new File("data/femur_target.stl")).get //.transform(p=>p+tv)

//  val template = MeshIO.readMesh(new File("data/femur_tmp_51.stl")).get
//  val target = MeshIO.readMesh(new File("data/femur_tar_101.stl")).get

//  val gpmmFile = new File("data/femur_gpmm_51.h5")

//  val gpmmL: PointDistributionModel[_3D, TriangleMesh] = {
////    StatisticalModelIO.readStatisticalTriangleMeshModel3D(gpmmFile).getOrElse{
//    val ref = template
//    val zeroMean = Field(EuclideanSpace[_3D], (_: Point[_3D]) => EuclideanVector.zeros[_3D])
//    val k = DiagonalKernel(GaussianKernel[_3D](50) * 10, 3)
//    val gp = GaussianProcess[_3D, EuclideanVector[_3D]](zeroMean, k)
//    val lowRankGP = LowRankGaussianProcess.approximateGPCholesky(ref, gp, relativeTolerance = 1.0, interpolator = NearestNeighborInterpolator())
//    val model = PointDistributionModel[_3D, TriangleMesh](ref, lowRankGP).truncate(math.min(lowRankGP.rank, template.pointSet.numberOfPoints*2))
////    StatisticalModelIO.writeStatisticalTriangleMeshModel3D(model, gpmmFile)
//    model
//  }

  println(s"Template points: ${template.pointSet.numberOfPoints}, triangles: ${template.triangles.length}")
  println(s"Target points: ${target.pointSet.numberOfPoints}, triangles: ${target.triangles.length}")

  val scaledRef = template //.operations.decimate(200)
  val gpmm: PointDistributionModel[_3D, TriangleMesh] = {
    val ref = scaledRef
    val zeroMean = Field(EuclideanSpace[_3D], (_: Point[_3D]) => EuclideanVector.zeros[_3D])
    val k = DiagonalKernel(GaussianKernel[_3D](50), 3)
    val gp = GaussianProcess[_3D, EuclideanVector[_3D]](zeroMean, k)
    val lowRankGP = LowRankGaussianProcess.approximateGPCholesky(ref, gp, relativeTolerance = 0.01, interpolator = NearestNeighborInterpolator())
    val model = PointDistributionModel[_3D, TriangleMesh](ref, lowRankGP).truncate(math.min(lowRankGP.rank, template.pointSet.numberOfPoints*2))
    model
  }
  println(s"Model rank: ${gpmm.rank} with ${scaledRef.pointSet.numberOfPoints} points")
  val scaledTarget = target //.operations.decimate(scaledRef.pointSet.numberOfPoints*2)
  val cpd = new GPMMRegistration[_3D, TriangleMesh](gpmm, lambda = 1, w = 0.0, max_iterations = 30)

  val t10 = System.currentTimeMillis()
  val fitPars = cpd.register(scaledTarget, tolerance = 0.0000001)
  val t11 = System.currentTimeMillis()
  println(s"Fitting time: ${(t11 - t10) / 1000.0} sec.")

  val fit = gpmm.instance(fitPars)
  println(s"Fit points: ${fit.pointSet.numberOfPoints}, triangles: ${fit.triangles.length}")

  val ui = ScalismoUI()
  val dataGroup = ui.createGroup("data")
  ui.show(dataGroup, template, "template").color = Color.RED
  ui.show(dataGroup, target, "target").color = Color.GREEN
  ui.show(dataGroup, fit, "fit")
}
