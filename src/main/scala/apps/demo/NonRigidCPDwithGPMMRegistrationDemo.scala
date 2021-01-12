package apps.demo

import java.awt.Color
import java.io.File

import api.registration.{GPMMRegistration, NonRigidCPDRegistration}
import api.registration.cpd.NonRigidCPDwithGPMM
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.common.{EuclideanSpace, Field, UnstructuredPointsDomain}
import scalismo.geometry.{EuclideanVector, Point, _3D}
import scalismo.io.{MeshIO, StatisticalModelIO}
import scalismo.kernels.{DiagonalKernel, GaussianKernel}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.{GaussianProcess, LowRankGaussianProcess, PointDistributionModel, StatisticalMeshModel}
import scalismo.ui.api.ScalismoUI

object NonRigidCPDwithGPMMRegistrationDemo extends App {
  scalismo.initialize()

//  val decimatedPoints = 50

//  val template = MeshIO.readMesh(new File("data/femur_reference.stl")).get.operations.decimate(decimatedPoints)
//  val target = template
//  val target = MeshIO.readMesh(new File("data/femur_target.stl")).get.operations.decimate(decimatedPoints*2)

  val template = MeshIO.readMesh(new File("data/femur_tmp_51.stl")).get
  val target = MeshIO.readMesh(new File("data/femur_tar_101.stl")).get

  val gpmmFile = new File("data/femur_gpmm_51.h5")

  val gpmm: PointDistributionModel[_3D, TriangleMesh] = StatisticalModelIO.readStatisticalTriangleMeshModel3D(gpmmFile).getOrElse{
    val ref = template
    val zeroMean = Field(EuclideanSpace[_3D], (_: Point[_3D]) => EuclideanVector.zeros[_3D])
    val k = DiagonalKernel(GaussianKernel[_3D](50) * 1, 3)
    val gp = GaussianProcess[_3D, EuclideanVector[_3D]](zeroMean, k)
    val lowRankGP = LowRankGaussianProcess.approximateGPCholesky(ref, gp, relativeTolerance = 0.01, interpolator = NearestNeighborInterpolator())
    val model = PointDistributionModel[_3D, TriangleMesh](ref, lowRankGP)
    StatisticalModelIO.writeStatisticalTriangleMeshModel3D(model, gpmmFile)
    model
  }

  println(s"Template points: ${template.pointSet.numberOfPoints}, triangles: ${template.triangles.length}")
  println(s"Target points: ${target.pointSet.numberOfPoints}, triangles: ${target.triangles.length}")

  val cpd = new GPMMRegistration[_3D, TriangleMesh](gpmm, target, lambda = 2, w = 0.0, max_iterations = 100)


  val t10 = System.currentTimeMillis()
  val fit = cpd.register()
  val t11 = System.currentTimeMillis()
  println(s"Fitting time: ${(t11 - t10) / 1000.0} sec.")

  println(s"Fit points: ${fit.pointSet.numberOfPoints}, triangles: ${fit.triangles.length}")

  val ui = ScalismoUI()
  val dataGroup = ui.createGroup("data")
  ui.show(dataGroup, template, "template").color = Color.RED
  ui.show(dataGroup, target, "target").color = Color.GREEN
  ui.show(dataGroup, fit, "fit")
}
