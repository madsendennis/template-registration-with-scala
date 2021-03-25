package api.registration.icp

import api.registration.utils.NonRigidClosestPointRegistrator.ClosestPointTriangleMesh3D
import breeze.linalg.{DenseMatrix, DenseVector}
import breeze.numerics.{abs, pow}
import scalismo.common._
import scalismo.geometry._
import scalismo.mesh.TriangleMesh
import scalismo.registration.LandmarkRegistration
import scalismo.statisticalmodel.{DiscreteLowRankGaussianProcess, PointDistributionModel}

class NonRigidICPwithGPMMspecial(
                                                                        val gpmm: PointDistributionModel[_3D, TriangleMesh],
                                                                        val target: TriangleMesh[_3D],
                                                                      )(
                                                                        implicit val vectorizer: Vectorizer[Point[_3D]],
                                                                        canWarp: DomainWarp[_3D, TriangleMesh]
                                                                      ) {
  private val initialPars = DenseVector.zeros[Double](gpmm.rank)
  private val template = gpmm.reference
  private val mindist = template.pointSet.points.toSeq.map { p => (template.pointSet.findNClosestPoints(p, 2).last.point - p).norm }.min

  private val sigmaDefault = Seq(mindist * 5, mindist * 2) ++ (0 until 5).map(i => mindist / pow(10, i))

  def Registration(max_iteration: Int, tolerance: Double = 0.001, sigma2: Seq[Double] = sigmaDefault): PointDistributionModel[_3D, TriangleMesh] = {
    val finalGPMM = sigma2.zipWithIndex.foldLeft(gpmm) { (initGPMM, config) =>
      val s = config._1
      val j = config._2
      val innerFit = (0 until max_iteration).foldLeft((initGPMM, Double.PositiveInfinity)) { (it, i) =>
        val iter = Iteration(it._1, target, s)
        val distance = iter._2
        println(s"ICP, iteration: ${j * max_iteration + i}/${max_iteration * sigma2.length}, sigma2: ${s}, average distance to target: ${distance}")
        val newGPMM = iter._1
        val diff = abs(distance - it._2)
        if (diff < tolerance) {
          println(s"Converged, difference in steps: ${diff}")
          return newGPMM
        } else {
          iter
        }
      }
      innerFit._1
    }
    finalGPMM
  }

  def getCorrespondence(template: TriangleMesh[_3D], target: TriangleMesh[_3D]): (Seq[(PointId, Point[_3D], Double)], Double) = {
    (Seq(), 0.0)
  }

  def updateModel(gpmmloc: PointDistributionModel[_3D, TriangleMesh], newRef: TriangleMesh[_3D]): PointDistributionModel[_3D, TriangleMesh] = {
    val combinedPoints = gpmmloc.reference.pointSet.points.toSeq zip newRef.pointSet.points.toSeq
    val globalRigidTransformation = LandmarkRegistration.rigid3DLandmarkRegistration(combinedPoints, Point(0,0,0))

    val transGPMM = gpmmloc.transform(globalRigidTransformation.inverse)
    val newGp = DiscreteLowRankGaussianProcess[_3D, TriangleMesh, EuclideanVector[_3D]](newRef,
      transGPMM.gp.meanVector,
      transGPMM.gp.variance,
      transGPMM.gp.basisMatrix)
    PointDistributionModel(newGp)
  }

  def Iteration(gpmm: PointDistributionModel[_3D, TriangleMesh], target: TriangleMesh[_3D], sigma2: Double): (PointDistributionModel[_3D, TriangleMesh], Double) = {
    val instance = gpmm.mean
    val (cpinfo, dist) = getCorrespondence(instance, target)
    val cpInit = cpinfo.filter(_._3 == 1.0).map(f => (f._1, f._2)).toIndexedSeq
    val m = gpmm.reference.pointSet.numberOfPoints
    val kernel = DenseMatrix.zeros[Double](m,m)
    (0 until m).foreach{i =>
      (0 until m).foreach { j =>
        kernel(i,j) = gpmm.gp.cov(PointId(i), PointId(j))(0,0)
      }
    }
//    val invKernel = KernelHelper(gpmm.reference).myPinv(kernel)
//    val Vpoints = gpmm.reference.pointSet.points.toIndexedSeq
//    val V = DiscreteDomainConverter.denseMatrixToTriangleMesh3D.toMatrix(gpmm.reference)
//    val Vupdate = DiscreteDomainConverter.denseMatrixToTriangleMesh3D.denseMatrixToDomain(V+invKernel*V, gpmm.reference).pointSet.points.toIndexedSeq
//    val Vdiff = Vpoints.zip(Vupdate).map{case (p,u) => (u-p)}.toIndexedSeq
//
//    val cp = cpInit.zip(Vdiff).map{case(cpId, d) => (cpId._1, cpId._2+d)}
    val cp = cpInit

    val tdata = cp.map{case(id, p) => (instance.pointSet.point(id), p)}
    val transform = LandmarkRegistration.rigid3DLandmarkRegistration(tdata, Point(0,0,0))

    val posteriorMean = gpmm.posterior(cp, sigma2).mean.transform(transform)
    val newGPMM = updateModel(gpmm, posteriorMean)
    (newGPMM, dist)
  }
}


class NonRigidICPwithGPMMTriangle3Dspecial(gpmm: PointDistributionModel[_3D, TriangleMesh],
                                    target: TriangleMesh[_3D]) extends NonRigidICPwithGPMMspecial(gpmm, target) {
  override def getCorrespondence(template: TriangleMesh[_3D], target: TriangleMesh[_3D]): (Seq[(PointId, Point[_3D], Double)], Double) = {
    ClosestPointTriangleMesh3D.closestPointCorrespondence(template, target)
  }
}

