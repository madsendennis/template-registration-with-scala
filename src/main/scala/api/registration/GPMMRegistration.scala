package api.registration

import api.registration.cpd.{NonRigidCPDwithGPMM, NonRigidCPDwithGPMMnew}
import api.registration.utils.PointSequenceConverter
import scalismo.common.{DiscreteDomain, DiscreteField, DomainWarp, Vectorizer}
import scalismo.geometry.{NDSpace, Point}
import scalismo.statisticalmodel.PointDistributionModel

import scala.language.higherKinds

class GPMMRegistration[D: NDSpace, DDomain[A] <: DiscreteDomain[A]](
    gpmm: PointDistributionModel[D, DDomain],
    target: DDomain[D],
    lambda: Double = 2,
    w: Double = 0,
    max_iterations: Int = 100)(implicit warper: DomainWarp[D, DDomain], vectorizer: Vectorizer[Point[D]], pointSequenceConverter: PointSequenceConverter[D]) {
  val cpd = new NonRigidCPDwithGPMMnew(gpmm, target, lambda, w, max_iterations)

  def registrationMethod(): DDomain[D] = cpd.Registration()

  def register(): DDomain[D] = {
    val template = gpmm.reference
    val registration = registrationMethod()
    val warpField = DiscreteField(template, template.pointSet.points.toIndexedSeq.zip(registration.pointSet.points.toIndexedSeq).map { case (a, b) => b - a })
    warper.transformWithField(template, warpField)
  }
}