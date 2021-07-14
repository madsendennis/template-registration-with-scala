package api.registration

import api.registration.cpd.CPDFactory
import api.registration.utils.PointSequenceConverter
import breeze.linalg.DenseVector
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

  def register(target: DDomain[D]): (DDomain[D], DenseVector[Double]) = {
    val registrationTask = registrationMethod(target.pointSet.points.toSeq)
    val registrationAll = registrationTask.Registration(max_iterations)
    val registration = registrationAll._1
    val P1 = registrationAll._2
    val warpField = DiscreteField(template, template.pointSet.points.toIndexedSeq.zip(registration).map { case (a, b) => b - a })
    (warper.transformWithField(template, warpField),P1)
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
