package api.registration.pipeline

import scalismo.common.DiscreteDomain

trait RegistrationMethod[D, DDomain[A] <: DiscreteDomain[A]] {
  def register(target: RegistrationTarget[D, DDomain]): RegistrationResult[D, DDomain]
}
