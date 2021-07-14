package apps

import api.registration.{AffineCPDRegistration, NonRigidCPDRegistration, RigidCPDRegistration}
import api.registration.pipeline.{HasLandmarks, RegistrationMethod, RegistrationResult, RegistrationTarget, RegistrationTask}
import apps.tasks.{FemurRegistration, FishRegistration}
import scalismo.geometry._3D
import scalismo.mesh.TriangleMesh

object RegistrationExperiment extends App {
  scalismo.initialize()

  val registrationTasks: Seq[RegistrationTask[_3D, TriangleMesh]] = Seq(
    FishRegistration("./data"),
    FemurRegistration("./data")
  )

  for (registrationTask <- registrationTasks) {
    val registrationMethods: Seq[RegistrationMethod[_3D, TriangleMesh]] = Seq(
      new RigidCPDRegistration(registrationTask.template) with RegistrationMethod[_3D, TriangleMesh] {
        override def register(target: RegistrationTarget[_3D, TriangleMesh]): RegistrationResult[_3D, TriangleMesh] =
          new RegistrationResult[_3D, TriangleMesh] {
            override def id: String = target.id

            override def method: String = "rigid_cpd"

            override def result: TriangleMesh[_3D] = register(target.target)._1
          }
      },
      new NonRigidCPDRegistration(registrationTask.template) with RegistrationMethod[_3D, TriangleMesh] {
        override def register(target: RegistrationTarget[_3D, TriangleMesh]): RegistrationResult[_3D, TriangleMesh] =
          new RegistrationResult[_3D, TriangleMesh] {
            override def id: String = target.id

            override def method: String = "nonrigid_cpd"

            override def result: TriangleMesh[_3D] = register(target.target)._1
          }
      },
      new AffineCPDRegistration(registrationTask.template) with RegistrationMethod[_3D, TriangleMesh] {
        override def register(target: RegistrationTarget[_3D, TriangleMesh]): RegistrationResult[_3D, TriangleMesh] =
          new RegistrationResult[_3D, TriangleMesh] {
            override def id: String = target.id

            override def method: String = "affine_cpd"

            override def result: TriangleMesh[_3D] = {
              // Note shows how one would look for landmarks, with an assert it could be required and passed in
              println("Has target landmarks? - " + target.isInstanceOf[RegistrationTarget[_3D, TriangleMesh] with HasLandmarks[_3D]])
              register(target.target)._1
            }
          }
      }
    )

    registrationTask.run(registrationMethods)
  }
}
