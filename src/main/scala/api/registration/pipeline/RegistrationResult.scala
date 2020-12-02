package api.registration.pipeline

import scalismo.common.DiscreteDomain

trait RegistrationResult[D, DDomain[D] <: DiscreteDomain[D]] {
  def id: String
  def method: String
  def result: DDomain[D]
}
