package api.registration.pipeline

import scalismo.common.DiscreteDomain

import scala.util.Try

trait RegistrationTask[D, DDomain[A] <: DiscreteDomain[A]] {
  def template: DDomain[D]
  def targets: Seq[RegistrationTarget[D, DDomain]]
  def saveRegistrationResult(registrationResult: RegistrationResult[D, DDomain]): Try[Unit]

  def run(registrationMethods: Seq[RegistrationMethod[D, DDomain]]) = {
    for (method <- registrationMethods) {
      for (target <- targets) {
        val result = method.register(target)
        saveRegistrationResult(result)
      }
    }
  }
}
