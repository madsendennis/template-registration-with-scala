package api.registration

import api.registration.cpd.NonRigidCPDwithGPMM
import api.registration.utils.PointSequenceConverter
import breeze.linalg.DenseVector
import scalismo.common.{DiscreteDomain, DiscreteField, DomainWarp, Vectorizer}
import scalismo.geometry.{NDSpace, Point, _3D}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.PointDistributionModel

import scala.language.higherKinds

class GPMMRegistration[D: NDSpace, DDomain[A] <: DiscreteDomain[A]](
    gpmm: PointDistributionModel[D, DDomain],
    lambda: Double = 2,
    w: Double = 0,
    max_iterations: Int = 100)(implicit warper: DomainWarp[D, DDomain], vectorizer: Vectorizer[Point[D]], pointSequenceConverter: PointSequenceConverter[D]) {
  val cpd = new NonRigidCPDwithGPMM(gpmm, lambda, w, max_iterations)

  def registrationMethod(target: DDomain[D], tolerance: Double): DenseVector[Double] = cpd.Registration(target.pointSet.points.toSeq, tolerance)

  def registerAndWarp(target: DDomain[D], tolerance: Double = 0.001): DDomain[D] = {
    val template = gpmm.reference
    val registrationPars = registrationMethod(target, tolerance)
    val registration = gpmm.instance(registrationPars)
    val warpField = DiscreteField(template, template.pointSet.points.toIndexedSeq.zip(registration.pointSet.points.toIndexedSeq).map { case (a, b) => b - a })
    warper.transformWithField(template, warpField)
  }

  def register(target: DDomain[D], tolerance: Double = 0.001): DenseVector[Double] = {
    registrationMethod(target, tolerance)
  }
}