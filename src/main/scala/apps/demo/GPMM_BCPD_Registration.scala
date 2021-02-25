package apps.demo

import java.awt.Color
import java.io.File

import api.registration.GpmmBcpdRegistration
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.geometry.{EuclideanVector, EuclideanVector3D, Point3D, _3D}
import scalismo.io.MeshIO
import scalismo.kernels.{DiagonalKernel, GaussianKernel}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.{GaussianProcess, LowRankGaussianProcess, PointDistributionModel}
import scalismo.transformations.{RigidTransformation, Rotation3D, Scaling, Translation3D, TranslationAfterRotation3D, TranslationAfterScalingAfterRotation}
import scalismo.ui.api.ScalismoUI

object GPMM_BCPD_Registration extends App {
  scalismo.initialize()

  val globalTrans = TranslationAfterScalingAfterRotation(Translation3D(EuclideanVector3D(20.0, 5.0, 5.0)), Scaling[_3D](1.5), Rotation3D(0.4, 0.0, 0.0, Point3D(0, 0, 0)))

  val template = MeshIO.readMesh(new File("data/femur_reference.stl")).get.operations.decimate(100)
  val target = MeshIO.readMesh(new File("data/femur_target.stl")).get.operations.decimate(100).transform(globalTrans)

  println(s"Template points: ${template.pointSet.numberOfPoints}, triangles: ${template.triangles.length}")
  println(s"Target points: ${target.pointSet.numberOfPoints}, triangles: ${target.triangles.length}")

  val gpmm: PointDistributionModel[_3D, TriangleMesh] = {
    val ref = template
    val k = DiagonalKernel(GaussianKernel[_3D](50), 3)
    val gp = GaussianProcess[_3D, EuclideanVector[_3D]](k)
    val lowRankGP = LowRankGaussianProcess.approximateGPCholesky(ref, gp, relativeTolerance = 0.01, interpolator = NearestNeighborInterpolator())
    val model = PointDistributionModel[_3D, TriangleMesh](ref, lowRankGP).truncate(math.min(lowRankGP.rank, template.pointSet.numberOfPoints * 2))
    model
  }

  println(s"Model rank: ${gpmm.rank} with ${template.pointSet.numberOfPoints} points")

  val cpd = new GpmmBcpdRegistration[_3D, TriangleMesh](gpmm,
    target,
    w = 0.0,
    lambda = 1,
    gamma = 1.0,
    k = 1000,
    max_iterations = 50
  )

  val t10 = System.currentTimeMillis()
  val (fitPars, fitTrans) = cpd.register(tolerance = 0.000001)
  val t11 = System.currentTimeMillis()
  println(s"Fitting time: ${(t11 - t10) / 1000.0} sec.")

  val myTrans = TranslationAfterScalingAfterRotation(fitTrans.t, fitTrans.s, fitTrans.R)
  val fit = gpmm.instance(fitPars).transform(myTrans)
  println(s"Fit points: ${fit.pointSet.numberOfPoints}, triangles: ${fit.triangles.length}")

  println(s"Final transformation, s: ${fitTrans.s.s}, t: ${fitTrans.t.t.toBreezeVector}, \n R: ${fitTrans.R.rotationMatrix}")

  val ui = ScalismoUI("BCPD as GPMM-regression")
  val dataGroup = ui.createGroup("data")
  ui.show(dataGroup, template, "template").color = Color.RED
  ui.show(dataGroup, target, "target").color = Color.GREEN
  ui.show(dataGroup, fit, "fit")
}
