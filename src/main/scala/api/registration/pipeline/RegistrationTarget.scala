package api.registration.pipeline

import scalismo.common.DiscreteDomain
import scalismo.geometry.Landmark

trait RegistrationTarget[D, DDomain[A] <: DiscreteDomain[A]] {
  def id: String
  def target: DDomain[D]
}

trait HasLandmarks[D] {
  def landmarks: Seq[Landmark[D]]
}

trait RegistrationTargetWithLandmarks[D, DDomain[D] <: DiscreteDomain[D]] extends RegistrationTarget[D, DDomain] with HasLandmarks[D]
