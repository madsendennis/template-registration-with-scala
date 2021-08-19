package other.icp

import api.registration.utils.Registrator
import scalismo.common._
import scalismo.geometry.{NDSpace, Point}

private[icp] class AffineICP[D: NDSpace](
                                          override val targetPoints: UnstructuredPoints[D],
                                          override val icp: ICPFactory[D]
                                        )(
                                          implicit vectorizer: Vectorizer[Point[D]],
                                          registrator: Registrator[D]
                                        ) extends RigidICP[D](targetPoints, icp) {
}
