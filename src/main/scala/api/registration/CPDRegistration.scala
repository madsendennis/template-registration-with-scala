package api.registration

import api.registration.cpd.CPDFactory
import api.registration.utils.PointSequenceConverter
import scalismo.common.{DiscreteDomain, DiscreteField, DomainWarp, Vectorizer}
import scalismo.geometry.{NDSpace, Point}

import language.higherKinds

class RigidCPDRegistration[D: NDSpace, DDomain[A] <: DiscreteDomain[A]](
    template: DDomain[D],
    lambda: Double = 2,
    beta: Double = 2,
    w: Double = 0,
    max_iterations: Int = 100)(implicit warper: DomainWarp[D, DDomain], vectorizer: Vectorizer[Point[D]], pointSequenceConverter: PointSequenceConverter[D]) {
  val cpd = new CPDFactory(template.pointSet.points.toSeq, lambda, beta, w)

  def registrationMethod(targetPoints: Seq[Point[D]]) = cpd.registerRigidly(targetPoints)

  def register(target: DDomain[D]): DDomain[D] = {
    val registrationTask = registrationMethod(target.pointSet.points.toSeq)
    val registration = registrationTask.Registration(max_iterations)
    val warpField = DiscreteField(target, target.pointSet.points.toIndexedSeq.zip(registration).map { case (a, b) => b - a })
    warper.transformWithField(target, warpField)
  }
}

class NonRigidCPDRegistration[D: NDSpace, DDomain[A] <: DiscreteDomain[A]](
    template: DDomain[D],
    lambda: Double = 2,
    beta: Double = 2,
    w: Double = 0,
    max_iterations: Int = 100)(implicit warper: DomainWarp[D, DDomain], vectorizer: Vectorizer[Point[D]], pointSequenceConverter: PointSequenceConverter[D])
    extends RigidCPDRegistration[D, DDomain](template, lambda, beta, w, max_iterations) {

  override def registrationMethod(targetPoints: Seq[Point[D]]) = cpd.registerNonRigidly(targetPoints)
}

class AffineCPDRegistration[D: NDSpace, DDomain[A] <: DiscreteDomain[A]](
    template: DDomain[D],
    lambda: Double = 2,
    beta: Double = 2,
    w: Double = 0,
    max_iterations: Int = 100)(implicit warper: DomainWarp[D, DDomain], vectorizer: Vectorizer[Point[D]], pointSequenceConverter: PointSequenceConverter[D])
    extends RigidCPDRegistration[D, DDomain](template, lambda, beta, w, max_iterations) {

  override def registrationMethod(targetPoints: Seq[Point[D]]) = cpd.registerAffine(targetPoints)
}