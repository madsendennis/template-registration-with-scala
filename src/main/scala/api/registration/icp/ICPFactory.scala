package api.registration.icp

import api.registration.utils.PointSequenceConverter
import scalismo.common.{PointSet, Vectorizer}
import scalismo.geometry.{NDSpace, Point}

/*
 Implementation of Point Set Registration: Coherent Point Drift
 In this script, only the non-rigid algorithm is implemented. Paper: https://arxiv.org/pdf/0905.2635.pdf
 A python implementation already exists from where parts of the implementation is from: https://github.com/siavashk/pycpd
 */
class ICPFactory[D: NDSpace](
                              val templatePoints: PointSet[D],
                            )(
                              implicit val vectorizer: Vectorizer[Point[D]],
                              dataConverter: PointSequenceConverter[D]
                            ) {
  val M: Int = templatePoints.numberOfPoints // num of template points
  val dim: Int = vectorizer.dim // dimension
  val template: PointSet[D] = templatePoints

  def registerRigidly(targetPoints: PointSet[D]): RigidICP[D] = {
    new RigidICP[D](targetPoints, this)
  }

  def registerNonRigidly(targetPoints: PointSet[D]): RigidICP[D] = {
    new NonRigidICP[D](targetPoints, this)
  }

  def registerAffine(targetPoints: PointSet[D]): RigidICP[D] = {
    new AffineICP[D](targetPoints, this)
  }

}
