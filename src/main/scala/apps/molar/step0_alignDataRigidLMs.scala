package apps.molar

import java.io.File

import apps.util.AlignmentTransforms
import scalismo.geometry.Point3D
import scalismo.io.{LandmarkIO, MeshIO}
import scalismo.ui.util.FileUtil
import scalismo.utils.Random.implicits.randomGenerator

object step0_alignDataRigidLMs extends App{
  scalismo.initialize()

  val ref = MeshIO.readMesh(data.referenceMesh).get
  val refLms = LandmarkIO.readLandmarksJson3D(data.referenceLms).get

  val meshes = data.rawMeshes.listFiles(_.getName.endsWith(".vtk")).sorted

  data.alignedMeshesComplete.mkdirs()
  data.alignedLmsComplete.mkdirs()

  meshes.foreach{f =>
    println(s"Processing: ${f}")
    val baseName = FileUtil.basename(f)
    val fLms = new File(data.rawLms, baseName+".json")

    val m = MeshIO.readMesh(f).get
    val lm = LandmarkIO.readLandmarksJson3D(fLms).get

    val transform = AlignmentTransforms.computeTransform(lm, refLms, Point3D(0,0,0))

    MeshIO.writeMesh(m.transform(transform), new File(data.alignedMeshesComplete, baseName+".ply"))
    LandmarkIO.writeLandmarksJson(lm.map(_.transform(transform)), new File(data.alignedLmsComplete, fLms.getName))
  }

}
