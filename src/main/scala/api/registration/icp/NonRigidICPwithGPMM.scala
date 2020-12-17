package api.registration.icp

import api.registration.utils.NonRigidClosestPointRegistrator._
import breeze.linalg.DenseVector
import breeze.numerics.abs
import scalismo.common.{DiscreteDomain, PointId, UnstructuredPointsDomain, Vectorizer}
import scalismo.geometry._
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.PointDistributionModel

class NonRigidICPwithGPMM[D: NDSpace, DDomain[D] <: DiscreteDomain[D]](
                                                                        val gpmm: PointDistributionModel[D, DDomain],
                                                                        val target: DDomain[D],
                                                                      )(
                                                                        implicit val vectorizer: Vectorizer[Point[D]]
                                                                      ) {
  private val initialPars = DenseVector.zeros[Double](gpmm.rank)

  def Registration(max_iteration: Int, tolerance: Double = 0.001): DDomain[D] = {
    val sigma2 = 1.0

    val fit = (0 until max_iteration).foldLeft((initialPars, Double.PositiveInfinity)) { (it, i) =>
      val iter = Iteration(it._1, target, sigma2)
      val distance = iter._2
      println(s"ICP, iteration: ${i}, distance: ${distance}")
      val pars = iter._1
      val diff = abs(distance - it._2)
      if (diff < tolerance) {
        println(s"Converged, difference in steps: ${diff}")
        return gpmm.instance(pars)
      } else {
        iter
      }
    }
    gpmm.instance(fit._1)
  }

  def getCorrespondence(template: DDomain[D], target: DDomain[D]): (Seq[(PointId, Point[D], Double)], Double) = {
    (Seq(), 0.0)
  }

  def Iteration(pars: DenseVector[Double], target: DDomain[D], sigma2: Double): (DenseVector[Double], Double) = {
    val instance = gpmm.instance(pars)
    val (cpinfo, dist) = getCorrespondence(instance, target)
    val cp = cpinfo.filter(_._3 == 1.0).map(f => (f._1, f._2)).toIndexedSeq
    val posteriorMean = gpmm.posterior(cp, sigma2).mean
    (gpmm.coefficients(posteriorMean), dist)
  }
}


class NonRigidICPwithGPMMTriangle3D(gpmm: PointDistributionModel[_3D, TriangleMesh],
                                    target: TriangleMesh[_3D]) extends NonRigidICPwithGPMM[_3D, TriangleMesh](gpmm, target) {
  override def getCorrespondence(template: TriangleMesh[_3D], target: TriangleMesh[_3D]): (Seq[(PointId, Point[_3D], Double)], Double) = {
    ClosestPointTriangleMesh3D.closestPointCorrespondence(template, target)
  }
}

class NonRigidICPwithGPMMTriangle3DNormalDirection(gpmm: PointDistributionModel[_3D, TriangleMesh],
                                                   target: TriangleMesh[_3D]) extends NonRigidICPwithGPMM[_3D, TriangleMesh](gpmm, target) {
  override def getCorrespondence(template: TriangleMesh[_3D], target: TriangleMesh[_3D]): (Seq[(PointId, Point[_3D], Double)], Double) = {
    ClosestPointAlongNormalTriangleMesh3D.closestPointCorrespondence(template, target)
  }
}

class NonRigidICPwithGPMMUnstructuredPointsDomain3D(gpmm: PointDistributionModel[_3D, UnstructuredPointsDomain],
                                                    target: UnstructuredPointsDomain[_3D]) extends NonRigidICPwithGPMM[_3D, UnstructuredPointsDomain](gpmm, target) {
  override def getCorrespondence(template: UnstructuredPointsDomain[_3D], target: UnstructuredPointsDomain[_3D]): (Seq[(PointId, Point[_3D], Double)], Double) = {
    ClosestPointUnstructuredPointsDomain3D.closestPointCorrespondence(template, target)
  }
}

class NonRigidICPwithGPMMUnstructuredPointsDomain2D(gpmm: PointDistributionModel[_2D, UnstructuredPointsDomain],
                                                    target: UnstructuredPointsDomain[_2D]) extends NonRigidICPwithGPMM[_2D, UnstructuredPointsDomain](gpmm, target) {
  override def getCorrespondence(template: UnstructuredPointsDomain[_2D], target: UnstructuredPointsDomain[_2D]): (Seq[(PointId, Point[_2D], Double)], Double) = {
    ClosestPointUnstructuredPointsDomain2D.closestPointCorrespondence(template, target)
  }
}

class NonRigidICPwithGPMMUnstructuredPointsDomain1D(gpmm: PointDistributionModel[_1D, UnstructuredPointsDomain],
                                                    target: UnstructuredPointsDomain[_1D]) extends NonRigidICPwithGPMM[_1D, UnstructuredPointsDomain](gpmm, target) {
  override def getCorrespondence(template: UnstructuredPointsDomain[_1D], target: UnstructuredPointsDomain[_1D]): (Seq[(PointId, Point[_1D], Double)], Double) = {
    ClosestPointUnstructuredPointsDomain1D.closestPointCorrespondence(template, target)
  }
}

