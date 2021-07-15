package apps.demo

import java.awt.Color
import java.io.File

import api.registration.GpmmCpdRegistration
import scalismo.common.{EuclideanSpace, RealSpace}
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.geometry.{EuclideanVector, Point, _3D}
import scalismo.io.MeshIO
import scalismo.kernels.{DiagonalKernel, PDKernel}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.{GaussianProcess, LowRankGaussianProcess, PointDistributionModel}
import scalismo.ui.api.ScalismoUI

// Uses 2*sigma^2 in comparison to the default Gaussian Kernel in Scalismo
case class RealGaussianKernel[D](sigma: Double) extends PDKernel[D] {
  val sigma2 = sigma * sigma

  override def domain = EuclideanSpace[D]

  override def k(x: Point[D], y: Point[D]): Double = {
    val r = x - y
    scala.math.exp(-r.norm2 / (2 * sigma2))
  }
}

object GPMM_CPD_Registration extends App {
  scalismo.initialize()

  val template = MeshIO.readMesh(new File("data/femur_reference.stl")).get
  val target = MeshIO.readMesh(new File("data/femur_target.stl")).get

  println(s"Template points: ${template.pointSet.numberOfPoints}, triangles: ${template.triangles.length}")
  println(s"Target points: ${target.pointSet.numberOfPoints}, triangles: ${target.triangles.length}")

  val gpmm: PointDistributionModel[_3D, TriangleMesh] = {
    val ref = template
    val k = DiagonalKernel(RealGaussianKernel[_3D](50), 3)
    val gp = GaussianProcess[_3D, EuclideanVector[_3D]](k)
    val lowRankGP = LowRankGaussianProcess.approximateGPCholesky(ref, gp, relativeTolerance = 0.01, interpolator = NearestNeighborInterpolator())
    val model = PointDistributionModel[_3D, TriangleMesh](ref, lowRankGP).truncate(math.min(lowRankGP.rank, template.pointSet.numberOfPoints * 2))
    model
  }

  println(s"Model rank: ${gpmm.rank} with ${template.pointSet.numberOfPoints} points")

//  val cpd = new GpmmCpdRegistration[_3D, TriangleMesh](gpmm, target, Seq(), Seq(), lambda = 1, w = 0.0, max_iterations = 30)
//
//  val t10 = System.currentTimeMillis()
//  val fitPars = cpd.register(tolerance = 0.0000001)
//  val t11 = System.currentTimeMillis()
//  println(s"Fitting time: ${(t11 - t10) / 1000.0} sec.")
//
//  val fit = gpmm.instance(fitPars)
//  println(s"Fit points: ${fit.pointSet.numberOfPoints}, triangles: ${fit.triangles.length}")
//
//  val ui = ScalismoUI()
//  val dataGroup = ui.createGroup("data")
//  ui.show(dataGroup, template, "template").color = Color.RED
//  ui.show(dataGroup, target, "target").color = Color.GREEN
//  ui.show(dataGroup, fit, "fit")
}
