package api.registration

import scalismo.common.UnstructuredPointsDomain
import scalismo.geometry.{Dim, Landmark, _3D}
import scalismo.io.MeshIO
import scalismo.mesh.{TriangleMesh, TriangleMesh3D}

import scala.reflect.io.Path
import scala.util.Try

trait RegistrationTarget[D <: Dim] {
  def id: String
  def mesh: TriangleMesh[D]
}

trait HasLandmakrs[D <: Dim] {
  def landmarks: Seq[Landmark[D]]
}

trait RegistrationTargetWithLandmarks[D <: Dim] extends RegistrationTarget[D] with HasLandmakrs[D]

trait RegistrationResult[D <: Dim] {
  def id: String
  def method: String
  def mesh: TriangleMesh[D]
}

trait TemplateRegistrationTask[D <: Dim] {
  def template: TriangleMesh[D]
  def targets: Seq[RegistrationTarget[D]]
  def saveRegistrationResult(registrationResult: RegistrationResult[D]): Try[Unit]
}

trait CanRegister[D <: Dim] {
  def register(target: RegistrationTarget[D]): RegistrationResult[D]
}

class CPDRegistartion(template: TriangleMesh3D, alpha: Double, beta: Double) extends CanRegister[_3D] {

  val templatePoints = UnstructuredPointsDomain.Create.CreateUnstructuredPointsDomain3D.create(template.pointSet.points.toIndexedSeq)

  override def register(target: RegistrationTarget[_3D]): RegistrationResult[_3D] = {

    val targetPoints = UnstructuredPointsDomain.Create.CreateUnstructuredPointsDomain3D.create(target.mesh.pointSet.points.toIndexedSeq)
    val cpd = new CPDNonRigid(targetPoints, templatePoints, lamdba = 2, beta = 2, w = 0.0) // target, source ... Source is moving!!!
    val finalReg = cpd.Registration(max_iteration = 20)
    val registeredMesh = template.copy(pointSet = finalReg.pointSet)

    new RegistrationResult[_3D] {
      override def id: String = target.id

      override def method: String = "CPD"

      override def mesh: TriangleMesh[_3D] = registeredMesh
    }
  }
}

object CPDRegistartion {
  def apply(template: TriangleMesh3D, alpha: Double = 0.2, beta: Double = 0.7) = new CPDRegistartion(template, alpha, beta)
}

class ICPRegistartion(template: TriangleMesh3D, chi: Float, kappa: Int) extends CanRegister[_3D] {

  override def register(_target: RegistrationTarget[_3D]): RegistrationResult[_3D] = {
    assert(_target.isInstanceOf[RegistrationTargetWithLandmarks[_3D]], "The registration with ICP needs some landmarks for the target.")
    val target = _target.asInstanceOf[RegistrationTargetWithLandmarks[_3D]]

    new RegistrationResult[_3D] {
      override def id: String = target.id

      override def method: String = "CPD"

      override def mesh: TriangleMesh[_3D] = target.mesh
    }
  }
}

object ICPRegistartion {
  def apply(template: TriangleMesh3D, chi: Float = 0.2f, kappa: Int = 7) = new CPDRegistartion(template, chi, kappa)
}

case class FishRegistration(dataLocation: String) extends TemplateRegistrationTask[_3D] {
  val fishes = Path(dataLocation).toDirectory.files.filter(f => f.name.startsWith("fish")).toIndexedSeq.sortBy(_.name)

  override def template: TriangleMesh[_3D] = MeshIO.readMesh(fishes.head.jfile).get

  override def targets: Seq[RegistrationTarget[_3D]] = fishes.tail.map { fish =>
    new RegistrationTarget[_3D] {
      def id = fish.name
      def mesh = MeshIO.readMesh(fish.jfile).get
    }
  }

  val outputDirectory = Path(dataLocation) / "registered"
  override def saveRegistrationResult(registrationResult: RegistrationResult[_3D]): Try[Unit] = {
    val outDir = outputDirectory / registrationResult.method
    if (!outDir.exists) {
      java.nio.file.Files.createDirectories(outDir.jfile.toPath)
    }
    MeshIO.writeMesh(registrationResult.mesh, (outDir / registrationResult.id).jfile)
  }
}

object TestApp extends App {
  scalismo.initialize()

  val registrationTask: TemplateRegistrationTask[_3D] = FishRegistration("./data")

  val registrationMethods: Seq[CanRegister[_3D]] = Seq(
    CPDRegistartion(registrationTask.template) /*,
    ICPRegistartion(registrationTask.template)*/
  )

  for (method <- registrationMethods) {
    for (target <- registrationTask.targets) {
      val result = method.register(target)
      registrationTask.saveRegistrationResult(result)
    }
  }
}
