package apps

import java.io.File

import scalismo.geometry.{_3D, Landmark, Point}
import scalismo.io.{LandmarkIO, MeshIO, StatisticalModelIO}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.PointDistributionModel
import scalismo.transformations.{TranslationAfterRotation, TranslationAfterRotationSpace3D}
import scalismo.utils.Random

object DemoDatasetLoader {

  val dataPath = new File("data")
  val pathFemur = new File(dataPath, "femur")
  val pathArmadillo = new File(dataPath, "armadillo")

  def referenceFemur(): (TriangleMesh[_3D], Seq[Landmark[_3D]]) = {
    val reference = MeshIO.readMesh(new File(pathFemur, "femur_reference.stl")).get
    val landmarks = LandmarkIO.readLandmarksJson3D(new File(pathFemur, "femur_reference.json")).get
    (reference, landmarks)
  }

  def modelFemur(): (PointDistributionModel[_3D, TriangleMesh], Seq[Landmark[_3D]]) = {
    val model = StatisticalModelIO.readStatisticalTriangleMeshModel3D(new File(pathFemur, "femur_gp_model_50-components.h5")).get
    val landmarks = LandmarkIO.readLandmarksJson3D(new File(pathFemur, "femur_reference.json")).get
    (model, landmarks)
  }

  def targetFemur(offset: TranslationAfterRotation[_3D] = TranslationAfterRotationSpace3D(Point(0, 0, 0)).identityTransformation)(implicit
    rnd: Random): (TriangleMesh[_3D], Seq[Landmark[_3D]]) = {
    val target = MeshIO.readMesh(new File(pathFemur, "femur_target.stl")).get.transform(offset)
    val landmarks = LandmarkIO.readLandmarksJson3D(new File(pathFemur, "femur_target.json")).get.map(_.transform(offset))
    (target, landmarks)
  }

  def referenceArmadillo(): (TriangleMesh[_3D], Seq[Landmark[_3D]]) = {
    val reference = MeshIO.readMesh(new File(pathArmadillo, "armadillo.ply")).get
    val landmarks = LandmarkIO.readLandmarksJson3D(new File(pathArmadillo, "armadillo.json")).get
    (reference, landmarks)
  }

  def modelArmadillo(): (PointDistributionModel[_3D, TriangleMesh], Seq[Landmark[_3D]]) = {
    val model = StatisticalModelIO.readStatisticalTriangleMeshModel3D(new File(pathArmadillo, "armadillo-invlapdot-1000.h5")).get
    val landmarks = LandmarkIO.readLandmarksJson3D(new File(pathArmadillo, "armadillo.json")).get
    (model, landmarks)
  }

  def targetArmadillo(offset: TranslationAfterRotation[_3D] = TranslationAfterRotationSpace3D(Point(0, 0, 0)).identityTransformation)(implicit
    rnd: Random): (TriangleMesh[_3D], Seq[Landmark[_3D]]) = {
    val target = MeshIO.readMesh(new File(pathArmadillo, "armadillo_karate.ply")).get.transform(offset)
    val landmarks = LandmarkIO.readLandmarksJson3D(new File(pathArmadillo, "armadillo_karate.json")).get.map(_.transform(offset))
    (target, landmarks)
  }
}
