package api.registration.icp

import api.registration.utils.PointSequenceConverter
import scalismo.common._
import scalismo.geometry.{NDSpace, Point}

private[icp] class AffineICP[D: NDSpace](
                                          override val targetPoints: PointSet[D],
                                          override val icp: ICPFactory[D]
                                        )(
                                          implicit vectorizer: Vectorizer[Point[D]],
                                          dataConverter: PointSequenceConverter[D]
                                        ) extends RigidICP[D](targetPoints, icp) {
}
