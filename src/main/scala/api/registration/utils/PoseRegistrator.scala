package api.registration.utils

import scalismo.common.UnstructuredPoints
import scalismo.geometry.{EuclideanVector, Point, _2D, _3D}
import scalismo.registration.LandmarkRegistration

trait Registrator[D] {
  def register(points: Seq[(Point[D], Point[D])]): UnstructuredPoints[D]
}

object PoseRegistrator {

  private def computeRotationCenter[D](points: Seq[Point[D]]): Point[D] ={
    val center = points.map(_.toVector).reduce(_ + _) * 1.0 / points.length.toDouble
    center.toPoint
  }

  implicit object RigidRegistrator3D extends Registrator[_3D] {
    override def register(points: Seq[(Point[_3D], Point[_3D])]): UnstructuredPoints[_3D] = {
      val rotationCenter = computeRotationCenter(points.map(_._2))
      val t = LandmarkRegistration.rigid3DLandmarkRegistration(points, rotationCenter)
      UnstructuredPoints(points.map { case (p, _) => t.f(p) }.toIndexedSeq)
    }
  }

  implicit object AffineRegistrator3D extends Registrator[_3D] {
    override def register(points: Seq[(Point[_3D], Point[_3D])]): UnstructuredPoints[_3D] = {
      val rotationCenter = computeRotationCenter(points.map(_._2))
      val t = LandmarkRegistration.similarity3DLandmarkRegistration(points, rotationCenter)
      UnstructuredPoints(points.map { case (p, _) => t.f(p) }.toIndexedSeq)
    }
  }

  implicit object RigidRegistrator2D extends Registrator[_2D] {
    override def register(points: Seq[(Point[_2D], Point[_2D])]): UnstructuredPoints[_2D] = {
      val rotationCenter = computeRotationCenter(points.map(_._2))
      val t = LandmarkRegistration.rigid2DLandmarkRegistration(points, rotationCenter)
      UnstructuredPoints(points.map { case (p, _) => t.f(p) }.toIndexedSeq)
    }
  }

  implicit object AffineRegistrator2D extends Registrator[_2D] {
    override def register(points: Seq[(Point[_2D], Point[_2D])]): UnstructuredPoints[_2D] = {
      val rotationCenter = computeRotationCenter(points.map(_._2))
      val t = LandmarkRegistration.similarity2DLandmarkRegistration(points, rotationCenter)
      UnstructuredPoints(points.map { case (p, _) => t.f(p) }.toIndexedSeq)
    }
  }
}
