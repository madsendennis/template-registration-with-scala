package api.registration.utils

import scalismo.common.{DiscreteDomain, PointId, UnstructuredPoints, UnstructuredPointsDomain}
import scalismo.geometry.{EuclideanVector, Point, _1D, _2D, _3D}
import scalismo.mesh.TriangleMesh

trait ClosestPointDirection
case object ReferenceToTarget extends ClosestPointDirection
case object TargetToReference extends ClosestPointDirection
case object RandomDirection extends ClosestPointDirection

trait ClosestPointRegistrator[D, DDomain[D] <: DiscreteDomain[D]] {
  /*
  returns: Seq of point on template to corresponding point on target + a weight identifying the robustness of the closest point (1.0 = robust, 0.0 = not-robust)
           Additionally the average closest point distance is returned
   */
  def closestPointCorrespondence(reference: DDomain[D], target: DDomain[D], direction: ClosestPointDirection = ReferenceToTarget): (Seq[(PointId, Point[D], Double)], Double)
}

object NonRigidClosestPointRegistrator {

  // Todo: Different ICP "flavours", closest point in pointset, closest point on surface, closest point along normal
  // Todo: Swap directions

  private def isPointOnBoundary(id: PointId, mesh: TriangleMesh[_3D]): Boolean = {
    mesh.operations.pointIsOnBoundary(id)
  }

  private def isNormalDirectionOpposite(n1: EuclideanVector[_3D], n2: EuclideanVector[_3D]): Boolean = {
    // Todo: Add angle hyperparameter - currently it only looks if the vectors are opposite
    (n1 dot n2) < 0
  }

  private def isClosestPointIntersecting(id: PointId, cp: Point[_3D], mesh: TriangleMesh[_3D]): Boolean = {
    val p = mesh.pointSet.point(id)
    val v = p-cp
    val intersectingPoints = mesh.operations.getIntersectionPoints(p, v).filter(f => f != p) // All intersecting points with the closest point vector
    val closestIntersectingPoint = if (intersectingPoints.nonEmpty) intersectingPoints.map(ip => (p - ip).norm).min else Double.PositiveInfinity // Distance to closest intersecting point on template
    (closestIntersectingPoint < (v).norm)
  }

  object ClosestPointTriangleMesh3D extends ClosestPointRegistrator[_3D, TriangleMesh] {
    override def closestPointCorrespondence(reference: TriangleMesh[_3D], target: TriangleMesh[_3D], direction: ClosestPointDirection): (Seq[(PointId, Point[_3D], Double)], Double) = {
      var distance = 0.0

      def ref2tar: Seq[(PointId, Point[_3D], Double)] = {
        reference.pointSet.pointIds.toSeq.map { id =>
          val p = reference.pointSet.point(id)
          val closestPointOnSurface = target.operations.closestPointOnSurface(p)
          val closestPoint = target.pointSet.findClosestPoint(closestPointOnSurface.point)
          val w = if (isPointOnBoundary(closestPoint.id, target)) 0.0
          else if (isNormalDirectionOpposite(reference.vertexNormals.atPoint(id), target.vertexNormals.atPoint(closestPoint.id))) 0.0
          else if (isClosestPointIntersecting(id, closestPointOnSurface.point, reference)) 0.0
          else 1.0
          distance += closestPointOnSurface.distance
          (id, closestPointOnSurface.point, w)
        }
      }

      def tar2ref: Seq[(PointId, Point[_3D], Double)] = {
        target.pointSet.pointIds.toSeq.map { id =>
          val p = target.pointSet.point(id)
          val closestPointOnSurface = reference.operations.closestPointOnSurface(p)
          val closestPoint = reference.pointSet.findClosestPoint(closestPointOnSurface.point)
          val w = if (isPointOnBoundary(closestPoint.id, reference)) 0.0
          else if (isNormalDirectionOpposite(target.vertexNormals.atPoint(id), reference.vertexNormals.atPoint(closestPoint.id))) 0.0
          else if (isClosestPointIntersecting(id, closestPointOnSurface.point, target)) 0.0
          else 1.0
          distance += closestPointOnSurface.distance
          (closestPoint.id, p, w)
        }
      }

      val corr = direction match{
        case ReferenceToTarget => ref2tar
        case TargetToReference => tar2ref
        case RandomDirection => if (scala.util.Random.nextBoolean()) ref2tar else tar2ref
      }

      (corr, distance / reference.pointSet.numberOfPoints)
    }
  }

  object ClosestPointAlongNormalTriangleMesh3D extends ClosestPointRegistrator[_3D, TriangleMesh] {
    override def closestPointCorrespondence(reference: TriangleMesh[_3D], target: TriangleMesh[_3D], direction: ClosestPointDirection): (Seq[(PointId, Point[_3D], Double)], Double) = {
      var distance = 0.0
      val corr = reference.pointSet.pointIds.toSeq.map { id =>
        val p = reference.pointSet.point(id)
        val n = reference.vertexNormals.atPoint(id)

        val intersectingPoints = target.operations.getIntersectionPoints(p, n).filter(f => f != p)
        val closestPointAlongNormal = if (intersectingPoints.nonEmpty) Some(intersectingPoints.minBy(ip => (p - ip).norm)) else None

        val (closestPoint, w) = if(closestPointAlongNormal.nonEmpty) {
          val closestPoint = target.pointSet.findClosestPoint(closestPointAlongNormal.get)
          val weight = if (isPointOnBoundary(closestPoint.id, target)) 0.0
          else if (isNormalDirectionOpposite(reference.vertexNormals.atPoint(id), target.vertexNormals.atPoint(closestPoint.id))) 0.0
          else if (isClosestPointIntersecting(id, closestPointAlongNormal.get, reference)) 0.0
          else 1.0
          (closestPointAlongNormal.get, weight)
        }
        else (p, 0.0) // return p to avoid influincing the "distance" measure too much
        distance += (p - closestPoint).norm
        (id, closestPoint, w)
      }
      (corr, distance / reference.pointSet.numberOfPoints)
    }
  }

  object ClosestPointUnstructuredPointsDomain3DTarget extends ClosestPointRegistrator[_3D, UnstructuredPointsDomain] {
    override def closestPointCorrespondence(reference: UnstructuredPointsDomain[_3D], target: UnstructuredPointsDomain[_3D], direction: ClosestPointDirection): (Seq[(PointId, Point[_3D], Double)], Double) = {
      var distance = 0.0
      val corr = reference.pointSet.pointIds.toSeq.map { id =>
        val p = reference.pointSet.point(id)
        val closestPoint = target.pointSet.findClosestPoint(p)
        val w = 1.0
        distance += (p - closestPoint.point).norm
        (id, closestPoint.point, w)
      }
      (corr, distance / reference.pointSet.numberOfPoints)
    }
  }

    object ClosestPointUnstructuredPointsDomain3D extends ClosestPointRegistrator[_3D, UnstructuredPointsDomain] {
      override def closestPointCorrespondence(reference: UnstructuredPointsDomain[_3D], target: UnstructuredPointsDomain[_3D], direction: ClosestPointDirection): (Seq[(PointId, Point[_3D], Double)], Double) = {
        var distance = 0.0
        val corr = reference.pointSet.pointIds.toSeq.map { id =>
          val p = reference.pointSet.point(id)
          val closestPoint = target.pointSet.findClosestPoint(p)
          val w = 1.0
          distance += (p - closestPoint.point).norm
          (id, closestPoint.point, w)
        }
        (corr, distance / reference.pointSet.numberOfPoints)
      }
    }

  object ClosestPointUnstructuredPointsDomain2D extends ClosestPointRegistrator[_2D, UnstructuredPointsDomain] {
    override def closestPointCorrespondence(reference: UnstructuredPointsDomain[_2D], target: UnstructuredPointsDomain[_2D], direction: ClosestPointDirection): (Seq[(PointId, Point[_2D], Double)], Double) = {
      var distance = 0.0
      val corr = reference.pointSet.pointIds.toSeq.map { id =>
        val p = reference.pointSet.point(id)
        val closestPoint = target.pointSet.findClosestPoint(p)
        val w = 1.0
        distance += (p - closestPoint.point).norm
        (id, closestPoint.point, w)
      }
      (corr, distance / reference.pointSet.numberOfPoints)
    }
  }

  object ClosestPointUnstructuredPointsDomain1D extends ClosestPointRegistrator[_1D, UnstructuredPointsDomain] {
    override def closestPointCorrespondence(reference: UnstructuredPointsDomain[_1D], target: UnstructuredPointsDomain[_1D], direction: ClosestPointDirection): (Seq[(PointId, Point[_1D], Double)], Double) = {
      var distance = 0.0
      val corr = reference.pointSet.pointIds.toSeq.map { id =>
        val p = reference.pointSet.point(id)
        val closestPoint = target.pointSet.findClosestPoint(p)
        val w = 1.0
        distance += (p - closestPoint.point).norm
        (id, closestPoint.point, w)
      }
      (corr, distance / reference.pointSet.numberOfPoints)
    }
  }

}
