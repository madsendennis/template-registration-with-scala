package apps.tasks

import java.io.File

import api.registration.pipeline.{HasLandmarks, RegistrationResult, RegistrationTarget, RegistrationTask}
import scalismo.geometry.{Landmark, _3D}
import scalismo.io.{LandmarkIO, MeshIO}
import scalismo.mesh.TriangleMesh

import scala.reflect.io.Path
import scala.util.Try

case class FemurRegistration(dataLocation: String) extends RegistrationTask[_3D, TriangleMesh] {

  val femurs = Path(dataLocation).toDirectory.files.filter(f => f.name.startsWith("femur") && f.name.endsWith(".stl")).toIndexedSeq.sortBy(_.name)
  val outputDirectory = Path(dataLocation) / "registered"

  override def template: TriangleMesh[_3D] = MeshIO.readMesh(femurs.head.jfile).get

  override def targets: Seq[RegistrationTarget[_3D, TriangleMesh]] = femurs.tail.map { femur =>
    new RegistrationTarget[_3D, TriangleMesh] with HasLandmarks[_3D] {
      def id = femur.name
      def target = MeshIO.readMesh(femur.jfile).get
      override def landmarks: Seq[Landmark[_3D]] = LandmarkIO.readLandmarksJson3D(new File(femur.parent.jfile, femur.name.replace(".stl", ".json"))).get
    }
  }

  override def saveRegistrationResult(registrationResult: RegistrationResult[_3D, TriangleMesh]): Try[Unit] = {
    val outDir = outputDirectory / registrationResult.method
    if (!outDir.exists) {
      java.nio.file.Files.createDirectories(outDir.jfile.toPath)
    }
    MeshIO.writeMesh(registrationResult.result, (outDir / registrationResult.id.replace(".stl", ".ply")).jfile)
  }
}
