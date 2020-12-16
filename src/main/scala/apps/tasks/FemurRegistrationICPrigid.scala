package apps.tasks

import java.io.File

import api.registration.RigidICPRegistration
import api.registration.pipeline.{HasLandmarks, RegistrationMethod, RegistrationResult, RegistrationTarget, RegistrationTask}
import scalismo.common.{PointSet, PointWithId, UnstructuredPoints}
import scalismo.common.UnstructuredPoints.parametricToConcreteType3D
import scalismo.geometry.{EuclideanVector3D, Landmark, Point, Point3D, _3D}
import scalismo.io.{LandmarkIO, MeshIO}
import scalismo.mesh.{LineMesh2D, TriangleMesh, TriangleMesh2D}
import scalismo.mesh.boundingSpheres.ClosestPointWithType
import scalismo.transformations.{RigidTransformation, Rotation3D, Translation3D, TranslationAfterRotation3D}
import scalismo.ui.api.ScalismoUI

import scala.reflect.io.Path
import scala.util.Try

case class FemurRegistrationICPrigid(dataLocation: String) extends RegistrationTask[_3D, TriangleMesh] {

  private val rigidTrans: RigidTransformation[_3D] = TranslationAfterRotation3D(Translation3D(EuclideanVector3D(50.0, 20.0, 30.0)), Rotation3D(0.1, 0.2, 0.3, Point3D(0, 0, 0)))


  val femurs = Path(dataLocation).toDirectory.files.filter(f => f.name.startsWith("femur") && f.name.endsWith(".stl")).toIndexedSeq.sortBy(_.name)
  val outputDirectory = Path(dataLocation) / "registered"

  private val templateFile = femurs.find(_.name == "femur0_coarse.stl").get.jfile
  override def template: TriangleMesh[_3D] = MeshIO.readMesh(templateFile).get.transform(rigidTrans)

  private val targetFiles = femurs.find(_.name == "femur1_coarse.stl").get

  override def targets: Seq[RegistrationTarget[_3D, TriangleMesh]] = Seq(targetFiles).map { femur =>
    new RegistrationTarget[_3D, TriangleMesh] {
      def id = femur.name
      def target = MeshIO.readMesh(femur.jfile).get
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


object FemurRegistrationICPrigid extends App{
  scalismo.initialize()

  val femurReg = FemurRegistrationICPrigid("./data")
  val target = femurReg.targets.head.target

  val reg = new RigidICPRegistration(femurReg.template, 10)
  val fit = reg.register(target)

  val ui = ScalismoUI()
  val dataGroup = ui.createGroup("data")
  ui.show(dataGroup, femurReg.template, "template")
  ui.show(dataGroup, target, "template")
  ui.show(dataGroup, fit, "fit")
}
