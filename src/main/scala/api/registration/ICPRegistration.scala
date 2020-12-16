package api.registration

import api.registration.cpd.CPDFactory
import api.registration.icp.ICPFactory
import api.registration.utils.PointSequenceConverter
import scalismo.common.{DiscreteDomain, DiscreteField, DomainWarp, PointSet, PointWithId, Vectorizer}
import scalismo.geometry.{NDSpace, Point}
import scalismo.mesh.{TriangleMesh, TriangleMesh3D}

import scala.language.higherKinds

class RigidICPRegistration[D: NDSpace, DDomain[A] <: DiscreteDomain[A]](
    template: DDomain[D],
    max_iterations: Int = 100)(implicit warper: DomainWarp[D, DDomain], vectorizer: Vectorizer[Point[D]], pointSequenceConverter: PointSequenceConverter[D]) {
  val icp = new ICPFactory[D](template.pointSet)

  def registrationMethod(targetPoints: PointSet[D]) = icp.registerRigidly(targetPoints)

  def register(target: DDomain[D]): DDomain[D] = {
    val registrationTask = registrationMethod(target.pointSet)
    val registration = registrationTask.Registration(max_iterations)
    val warpField = DiscreteField(target, target.pointSet.points.toIndexedSeq.zip(registration.points.toIndexedSeq).map { case (a, b) => b - a })
    warper.transformWithField(target, warpField)
  }
}

//class NonRigidICPRegistration[D: NDSpace, DDomain[A] <: DiscreteDomain[A]](
//    template: DDomain[D],
//    max_iterations: Int = 100)(implicit warper: DomainWarp[D, DDomain], vectorizer: Vectorizer[Point[D]], pointSequenceConverter: PointSequenceConverter[D])
//    extends RigidICPRegistration[D, DDomain](template, max_iterations) {
//
//  override def registrationMethod(targetPoints: Seq[Point[D]]) = icp.registerNonRigidly(targetPoints)
//}
//
//class AffineICPRegistration[D: NDSpace, DDomain[A] <: DiscreteDomain[A]](
//    template: DDomain[D],
//    max_iterations: Int = 100)(implicit warper: DomainWarp[D, DDomain], vectorizer: Vectorizer[Point[D]], pointSequenceConverter: PointSequenceConverter[D])
//    extends RigidICPRegistration[D, DDomain](template, max_iterations) {
//
//  override def registrationMethod(targetPoints: Seq[Point[D]]) = icp.registerAffine(targetPoints)
//}
