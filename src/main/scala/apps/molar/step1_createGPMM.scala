package apps.molar

import api.registration.utils.{DotProductKernel, GaussianKernelParameters}
import scalismo.common.interpolation.TriangleMeshInterpolator3D
import scalismo.geometry.{EuclideanVector, _3D}
import scalismo.io.{MeshIO, StatisticalModelIO}
import scalismo.kernels.{DiagonalKernel, GaussianKernel}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.{GaussianProcess, LowRankGaussianProcess, PointDistributionModel}
import scalismo.ui.api.ScalismoUI

object step1_createGPMM extends App{
  scalismo.initialize()

  def minimumPointDistance(mesh: TriangleMesh[_3D]): Double = {
    mesh.pointSet.pointIds.toSeq.map{id =>
      (mesh.triangulation.adjacentPointsForPoint(id).map(nId => (mesh.pointSet.point(id).toVector-mesh.pointSet.point(nId).toVector).norm).min)
    }.min
  }

  def maximumPointDistance(mesh: TriangleMesh[_3D]): Double = {
    mesh.pointSet.pointIds.toSeq.map{id =>
      (mesh.triangulation.adjacentPointsForPoint(id).map(nId => (mesh.pointSet.point(id).toVector-mesh.pointSet.point(nId).toVector).norm).max)
    }.max
  }

  def avgPointDistance(mesh: TriangleMesh[_3D]): Double = {
    val allStuff = mesh.pointSet.pointIds.toSeq.map{id =>
      val neiP = mesh.triangulation.adjacentPointsForPoint(id)
      neiP.map(nId => (mesh.pointSet.point(id).toVector-mesh.pointSet.point(nId).toVector).norm).sum/neiP.length.toDouble
    }
    allStuff.sum/allStuff.length.toDouble
  }

  val ref = MeshIO.readMesh(data.referenceMesh).get
  println(s"Points on surface: ${ref.pointSet.numberOfPoints}")
  println(s"MinimumDist: ${minimumPointDistance(ref)}")
  println(s"Maximum: ${maximumPointDistance(ref)}")
  println(s"Average: ${avgPointDistance(ref)}")

  val md = ref.boundingBox.extent.norm
  println(s"Max distance: ${md}")

  val pars: Seq[GaussianKernelParameters] = Seq(GaussianKernelParameters(md/8.0, 10), GaussianKernelParameters(10.0, 8))
  val kernelLarge = GaussianKernel[_3D](md/3.0)*0.2
  val kernels = pars.map(p => GaussianKernel[_3D](p.sigma)*p.scaling)
  val kernel = kernels.tail.foldLeft(kernels.head)(_ + _)

  val DiagkernelDot = DiagonalKernel(DotProductKernel(kernelLarge, 1.0)*(1.0/md), 3)
//  val DiagKernelGauss = DiagonalKernel(kernel, 3)
  val Diagkernel = DiagkernelDot // + DiagKernelGauss
  val gp = GaussianProcess[_3D, EuclideanVector[_3D]](Diagkernel)

  val lowRankGP = LowRankGaussianProcess.approximateGPCholesky(ref, gp, relativeTolerance = 0.01, interpolator = TriangleMeshInterpolator3D[EuclideanVector[_3D]])
  val gpmm = PointDistributionModel[_3D, TriangleMesh](ref, lowRankGP)

  println(s"gpmm rank: ${gpmm.rank}")
  StatisticalModelIO.writeStatisticalTriangleMeshModel3D(gpmm, data.gpmmCoarse)

  val ui = ScalismoUI()
  ui.show(gpmm, "model")
}
