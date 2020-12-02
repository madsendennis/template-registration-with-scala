package api.registration

import api.registration.cpd.{CPDBase, RigidCPD}
import scalismo.common.{DiscreteDomain, DiscreteField, DomainWarp, UnstructuredPointsDomain, Vectorizer}
import scalismo.geometry.{Dim, Landmark, NDSpace, Point, _3D}
import scalismo.io.MeshIO
import scalismo.mesh.{TriangleMesh, TriangleMesh3D}

import language.higherKinds
import scala.reflect.io.Path
import scala.util.Try

trait RegistrationTarget[D, DDomain[A] <: DiscreteDomain[A]] {
  def id: String
  def target: DDomain[D]
}

trait HasLandmakrs[D] {
  def landmarks: Seq[Landmark[D]]
}

trait RegistrationTargetWithLandmarks[D, DDomain[D] <: DiscreteDomain[D]] extends RegistrationTarget[D, DDomain] with HasLandmakrs[D]

trait RegistrationResult[D, DDomain[D] <: DiscreteDomain[D]] {
  def id: String
  def method: String
  def result: DDomain[D]
}

trait TemplateRegistrationTask[D, DDomain[A] <: DiscreteDomain[A]] {
  def template: DDomain[D]
  def targets: Seq[RegistrationTarget[D, DDomain]]
  def saveRegistrationResult(registrationResult: RegistrationResult[D, DDomain]): Try[Unit]
}

trait CanRegister[D, DDomain[A] <: DiscreteDomain[A]] {
  def register(target: RegistrationTarget[D, DDomain]): RegistrationResult[D, DDomain]
}

case class FishRegistration(dataLocation: String) extends TemplateRegistrationTask[_3D, TriangleMesh] {
  val fishes = Path(dataLocation).toDirectory.files.filter(f => f.name.startsWith("fish")).toIndexedSeq.sortBy(_.name)

  override def template: TriangleMesh[_3D] = MeshIO.readMesh(fishes.head.jfile).get

  override def targets: Seq[RegistrationTarget[_3D, TriangleMesh]] = fishes.tail.map { fish =>
    new RegistrationTarget[_3D, TriangleMesh] {
      def id = fish.name
      def target = MeshIO.readMesh(fish.jfile).get
    }
  }

  val outputDirectory = Path(dataLocation) / "registered"

  override def saveRegistrationResult(registrationResult: RegistrationResult[_3D, TriangleMesh]): Try[Unit] = {
    val outDir = outputDirectory / registrationResult.method
    if (!outDir.exists) {
      java.nio.file.Files.createDirectories(outDir.jfile.toPath)
    }
    MeshIO.writeMesh(registrationResult.result, (outDir / registrationResult.id).jfile)
  }
}

object TestApp extends App {
  scalismo.initialize()

  val registrationTask: TemplateRegistrationTask[_3D, TriangleMesh] = FishRegistration("./data")

  val registrationMethods: Seq[CanRegister[_3D, TriangleMesh]] = Seq(
    new RigidCPDRegistration(registrationTask.template) with CanRegister[_3D, TriangleMesh] {
      override def register(target: RegistrationTarget[_3D, TriangleMesh]): RegistrationResult[_3D, TriangleMesh] =
        new RegistrationResult[_3D, TriangleMesh] {
          override def id: String = target.id
          override def method: String = "rigid_cpd"
          override def result: TriangleMesh[_3D] = register(target.target)
        }
    } /*,
    ICPRegistartion(registrationTask.template)*/
  )

  for (method <- registrationMethods) {
    for (target <- registrationTask.targets) {
      val result = method.register(target)
      registrationTask.saveRegistrationResult(result)
    }
  }
}
