package api.registration.icp

import api.registration.utils.{ClosestPointDirection, NonRigidClosestPointRegistrator, PoseRegistrator, ReferenceToTarget, Registrator}
import apps.util.AlignmentTransforms
import breeze.numerics.abs
import scalismo.common.{PointId, UnstructuredPoints, Vectorizer}
import scalismo.geometry.{NDSpace, Point, Point3D, _3D}
import scalismo.mesh.{TriangleMesh, TriangleMesh3D}
import scalismo.registration.LandmarkRegistration
import scalismo.transformations.TranslationAfterRotation

class RigidICPtransform(referenceInit: TriangleMesh[_3D],
                        target: TriangleMesh[_3D]){

  val center = (target.pointSet.points.toSeq.map(_.toVector).reduce(_ + _) * 1.0 / target.pointSet.numberOfPoints.toDouble).toPoint

  private def getFinalTransform(init: TriangleMesh[_3D], fin: TriangleMesh[_3D]): TranslationAfterRotation[_3D] = {
    val points = init.pointSet.points.toSeq.zip(fin.pointSet.points.toSeq).map{case(p1, p2) => (p1,p2)}
    LandmarkRegistration.rigid3DLandmarkRegistration(points, Point3D(0,0,0))
  }

  def Registration(max_iteration: Int, tolerance: Double = 0.001, direction: ClosestPointDirection = ReferenceToTarget): TranslationAfterRotation[_3D] = {
    val sigmaInit = 0.0

    val fit = (0 until max_iteration).foldLeft((referenceInit, sigmaInit)) { (it, i) =>
      val iter = Iteration(it._1, direction)
      val distance = iter._2
      println(s"ICP, iteration: ${i}, distance: ${distance}")
      val TY = iter._1
      val diff = abs(distance - it._2)
      if (diff < tolerance) {
        println("Converged")
        return getFinalTransform(referenceInit, TY)
      } else {
        iter
      }
    }
    getFinalTransform(referenceInit, fit._1)
  }

  def Iteration(reference: TriangleMesh[_3D], direction: ClosestPointDirection): (TriangleMesh[_3D], Double) = {
    val (cpinfo, dist) = NonRigidClosestPointRegistrator.ClosestPointTriangleMesh3D.closestPointCorrespondence(reference, target, direction) //(template, target)
    val cp = cpinfo.filter(_._3 == 1.0).map(f => (f._1, f._2)).toIndexedSeq
    val points = cp.map{case(id, p) => (reference.pointSet.point(id), p)}
//    val registeredPoints = PoseRegistrator.RigidRegistrator3D.register(points)
//    val moved = TriangleMesh3D(registeredPoints, reference.triangulation)

    val trans = LandmarkRegistration.rigid3DLandmarkRegistration(points, center)
    val moved = reference.transform(trans)
    (moved, dist)
  }

}
