package api.registration.icp

import api.registration.utils.PointSequenceConverter
import breeze.numerics.abs
import scalismo.common.{PointId, PointSet, Vectorizer}
import scalismo.geometry.{NDSpace, Point}

private[icp] class RigidICP[D: NDSpace](
                                         val targetPoints: PointSet[D],
                                         val icp: ICPFactory[D]
                                       )(
                                         implicit val vectorizer: Vectorizer[Point[D]],
                                         dataConverter: PointSequenceConverter[D]
                                       ) {
  require(vectorizer.dim == 2 || vectorizer.dim == 3)
  val N: Int = targetPoints.numberOfPoints
  val target: PointSet[D] = targetPoints


  def Registration(max_iteration: Int, tolerance: Double = 0.001): PointSet[D] = {
    val sigmaInit = 0.0

    val fit = (0 until max_iteration).foldLeft((icp.template, sigmaInit)) { (it, i) =>
      val currentSigma2 = it._2
      println(s"ICP, iteration: ${i}, variance: ${currentSigma2}")
      val iter = Iteration(target, it._1, it._2)
      val TY = iter._1
      val newSigma2 = iter._2
      val diff = abs(newSigma2 - currentSigma2)
      if (diff < tolerance) {
        println("Converged")
        return TY
      } else {
        iter
      }
    }
    fit._1
  }

  private def attributeCorrespondences(template: PointSet[D], target: PointSet[D]): Seq[(Point[D], Point[D])] = {
    val ptIds = template.pointIds.toIndexedSeq
    ptIds.map { id: PointId =>
      val pt = template.point(id)
      val closestPointOnMesh2 = target.findClosestPoint(pt).point
      (pt, closestPointOnMesh2)
    }
  }

  def Iteration(template: PointSet[D], target: PointSet[D], sigma2: Double): (PointSet[D], Double) = {
    //
    //    val correspondences: Seq[(Point[D], Point[D])] = attributeCorrespondences(template, target)
    //    val transformed: PointSet[D] = vectorizer.dim match {
    //      case 3 => {
    //        val transform = LandmarkRegistration.rigid3DLandmarkRegistration(correspondences, Point(0, 0, 0))
    //        template.transform(transform)
    //      }
    //      case 2 => LandmarkRegistration.rigid2DLandmarkRegistration(correspondences, Point(0,0))
    //    }
    //
    //    (transformed, sigma2)
    (template, sigma2)
  }

}
