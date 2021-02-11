package apps.demo

import java.awt.Color
import java.io.File

import api.registration.icp.{NonRigidICPwithGPMMTriangle3D, NonRigidOptimalStepICP}
import api.registration.utils.GPMMHelper
import breeze.linalg.DenseMatrix
import scalismo.common.interpolation.{NearestNeighborInterpolator, TriangleMeshInterpolator3D}
import scalismo.common.{EuclideanSpace, Field, PointId, RealSpace}
import scalismo.geometry.{EuclideanVector, NDSpace, Point, _3D}
import scalismo.io.MeshIO
import scalismo.kernels.{DiagonalKernel, GaussianKernel, MatrixValuedPDKernel, PDKernel}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.{DiscreteGaussianProcess, GaussianProcess, LowRankGaussianProcess, PointDistributionModel}
import scalismo.ui.api.ScalismoUI


case class AdjecentPointKernel[D](tmp: TriangleMesh[D], sigma: Double) extends PDKernel[D] {
  val sigma2 = sigma * sigma

  override def domain = RealSpace[D]

  override def k(x: Point[D], y: Point[D]): Double = {
    val pId1 = tmp.pointSet.findClosestPoint(x).id
    val pId2 = tmp.pointSet.findClosestPoint(y).id

    val adjacent1  =  tmp.triangulation.adjacentPointsForPoint(pId1)
    val adjacent2  =  tmp.triangulation.adjacentPointsForPoint(pId2)

    val numOfNeigh1 = adjacent1.length
    val numOfNeigh2 = adjacent2.length

        if(pId1 == pId2) numOfNeigh1 else if(adjacent1.contains(pId2)) -1.0 else 0.0 // L
//    if(pId1 == pId2) 1 else if(adjacent1.contains(pId2)) -1.0/(numOfNeigh1) else 0.0 // L_RM
//    if(pId1 == pId2) 1 else if(adjacent1.contains(pId2)) -1.0/(math.sqrt(numOfNeigh1*numOfNeigh2)) else 0.0 // L_sym
  }
}

object NonRigidICPRegistrationFunStuffWithOptimalStep extends App {
  scalismo.initialize()

  val dec = 50
  val template = MeshIO.readMesh(new File("data/femur_reference.stl")).get.operations.decimate(dec)
  val target = MeshIO.readMesh(new File("data/femur_target.stl")).get.operations.decimate(dec)

  val zeroMean = Field(EuclideanSpace[_3D], (_: Point[_3D]) => EuclideanVector.zeros[_3D])
  val k = DiagonalKernel(AdjecentPointKernel[_3D](template, 1.0), 3)
  val gp = GaussianProcess[_3D, EuclideanVector[_3D]](zeroMean, k)
  val lowRankGP = LowRankGaussianProcess.approximateGPCholesky(template, gp, relativeTolerance = 0.00, interpolator = TriangleMeshInterpolator3D[EuclideanVector[_3D]]())

  val gpmm = PointDistributionModel[_3D, TriangleMesh](template, lowRankGP)


//  val gpmm = GPMMHelper.automaticGPMMfromTemplate(template)

  println(s"GPMM rank: ${gpmm.rank}")

  println(s"Template points: ${template.pointSet.numberOfPoints}, triangles: ${template.triangles.length}")
  println(s"Target points: ${target.pointSet.numberOfPoints}, triangles: ${target.triangles.length}")

  val ui = ScalismoUI()
  val dataGroup = ui.createGroup("data")
  val modelGroup = ui.createGroup("model")
  ui.show(modelGroup, gpmm, "model")

  val nicp = new NonRigidICPwithGPMMTriangle3D(gpmm, target) // Without landmarks

  val t10 = System.currentTimeMillis()
  val alpha = Seq(1.0)
  val sig2 = alpha.map(f => f)
  val fit = nicp.Registration(100, 0.0000001, sigma2 = sig2)
  val t11 = System.currentTimeMillis()
  println(s"Fitting time: ${(t11 - t10) / 1000.0} sec.")

  println(s"Fit points: ${fit.pointSet.numberOfPoints}, triangles: ${fit.triangles.length}")


  ui.show(dataGroup, template, "template").color = Color.RED
  ui.show(dataGroup, target, "target").color = Color.GREEN
  ui.show(dataGroup, fit, "fit")
}
