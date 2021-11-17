package apps

import apps.gpmm.{GaussKernel, GaussMirrorKernel, InvLapDotKernel, InvLapKernel, SimpleTriangleModels3D, WhichKernel}

import java.io.File
import scalismo.geometry.{Landmark, _3D}
import scalismo.io.{LandmarkIO, MeshIO, StatisticalModelIO}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.PointDistributionModel
import scalismo.transformations.TranslationAfterRotation

trait DataSetLoader{
  def name: String
  def path: File
  def json: File = new File(path, "probabilisticFitting.json")
  def reference(): (TriangleMesh[_3D], Option[Seq[Landmark[_3D]]])
  def reference(decimate: Option[Int] = None): (TriangleMesh[_3D], Option[Seq[Landmark[_3D]]]) = {
    val (ref, lm) = reference()
    val dec = if(decimate.nonEmpty)
      ref.operations.decimate(decimate.get) else ref
    (dec, lm)
  }
  def target(offset: TranslationAfterRotation[_3D]): (TriangleMesh[_3D], Option[Seq[Landmark[_3D]]])
  def model(decimate: Option[Int] = None, kernelSelect: WhichKernel): (PointDistributionModel[_3D, TriangleMesh], Option[Seq[Landmark[_3D]]]) = {
    val (ref, lm) = reference()
    val (dec, decstring) = if(decimate.nonEmpty)
      (ref.operations.decimate(decimate.get), decimate.get.toString) else (ref, "full")
    val modelFile = new File(path, s"${name}_dec-${decstring}_${kernelSelect.name}_${kernelSelect.printpars}.h5")
    val model = StatisticalModelIO.readStatisticalTriangleMeshModel3D(modelFile).getOrElse {
      val m = SimpleTriangleModels3D.create(dec, kernelSelect)
      StatisticalModelIO.writeStatisticalTriangleMeshModel3D(m, modelFile).get
      m
    }
    (model, lm)
  }
  def modelGauss(decimate: Option[Int] = None, scaling: Double = 1.0, sigma: Double): (PointDistributionModel[_3D, TriangleMesh], Option[Seq[Landmark[_3D]]]) ={
    model(decimate, GaussKernel(scaling, sigma))
  }
  def modelGaussMirror(decimate: Option[Int] = None, scaling: Double = 1.0, sigma: Double): (PointDistributionModel[_3D, TriangleMesh], Option[Seq[Landmark[_3D]]]) ={
    model(decimate, GaussMirrorKernel(scaling, sigma))
  }
  def modelInvLap(decimate: Option[Int] = None, scaling: Double = 1.0): (PointDistributionModel[_3D, TriangleMesh], Option[Seq[Landmark[_3D]]]) ={
    model(decimate, InvLapKernel(scaling))
  }
  def modelInvLapDot(decimate: Option[Int] = None, scaling: Double = 1.0, gamma: Double): (PointDistributionModel[_3D, TriangleMesh], Option[Seq[Landmark[_3D]]]) ={
    model(decimate, InvLapDotKernel(scaling, gamma))
  }
}

object DemoDatasetLoader {
  val dataPath = new File("data")

  case object femur extends DataSetLoader{
    override def name: String = "femur"
    override def path: File = new File(dataPath, name)
    override def reference(): (TriangleMesh[_3D], Option[Seq[Landmark[_3D]]]) = {
      val reference = MeshIO.readMesh(new File(path, "femur.stl")).get
      val landmarks = LandmarkIO.readLandmarksJson3D(new File(path, "femur.json")).get
      (reference, Some(landmarks))
    }
    override def target(offset: TranslationAfterRotation[_3D]): (TriangleMesh[_3D], Option[Seq[Landmark[_3D]]]) = {
      val reference = MeshIO.readMesh(new File(path, "femur_target.stl")).get
      val landmarks = LandmarkIO.readLandmarksJson3D(new File(path, "femur_target.json")).get
      (reference, Some(landmarks))
    }
  }

  case object armadillo extends DataSetLoader{
    override def name: String = "armadillo"
    override def path: File = new File(dataPath, name)
    override def reference(): (TriangleMesh[_3D], Option[Seq[Landmark[_3D]]]) = {
      val reference = MeshIO.readMesh(new File(path, "armadillo.ply")).get
      val landmarks = LandmarkIO.readLandmarksJson3D(new File(path, "armadillo.json")).get
      (reference, Some(landmarks))
    }
    override def target(offset: TranslationAfterRotation[_3D]): (TriangleMesh[_3D], Option[Seq[Landmark[_3D]]]) = {
      val reference = MeshIO.readMesh(new File(path, "armadillo_karate.ply")).get
      val landmarks = LandmarkIO.readLandmarksJson3D(new File(path, "armadillo_karate.json")).get
      (reference, Some(landmarks))
    }
  }

  case object bunny extends DataSetLoader{
    override def name: String = "bunny"
    override def path: File = new File(dataPath, name)
    override def reference(): (TriangleMesh[_3D], Option[Seq[Landmark[_3D]]]) = {
      val reference = MeshIO.readMesh(new File(path, "bunny.ply")).get
      val landmarks = None
      (reference, landmarks)
    }
    override def target(offset: TranslationAfterRotation[_3D]): (TriangleMesh[_3D], Option[Seq[Landmark[_3D]]]) = {
      val reference = MeshIO.readMesh(new File(path, "armadillo_karate.ply")).get
      val landmarks = None
      (reference, landmarks)
    }
  }



//  def referenceFemur(): (TriangleMesh[_3D], Seq[Landmark[_3D]]) = {
//    val reference = MeshIO.readMesh(new File(pathFemur, "femur_reference.stl")).get
//    val landmarks = LandmarkIO.readLandmarksJson3D(new File(pathFemur, "femur_reference.json")).get
//    (reference, landmarks)
//  }
//
//  def modelFemur(): (PointDistributionModel[_3D, TriangleMesh], Seq[Landmark[_3D]]) = {
//    val model = StatisticalModelIO.readStatisticalTriangleMeshModel3D(new File(pathFemur, "femur_gp_model_50-components.h5")).get
//    val landmarks = LandmarkIO.readLandmarksJson3D(new File(pathFemur, "femur_reference.json")).get
//    (model, landmarks)
//  }
//
//  def targetFemur(offset: TranslationAfterRotation[_3D] = TranslationAfterRotationSpace3D(Point(0, 0, 0)).identityTransformation)(implicit
//    rnd: Random): (TriangleMesh[_3D], Seq[Landmark[_3D]]) = {
//    val target = MeshIO.readMesh(new File(pathFemur, "femur_target.stl")).get.transform(offset)
//    val landmarks = LandmarkIO.readLandmarksJson3D(new File(pathFemur, "femur_target.json")).get.map(_.transform(offset))
//    (target, landmarks)
//  }
//
//  def referenceArmadillo(): (TriangleMesh[_3D], Seq[Landmark[_3D]]) = {
//    val reference = MeshIO.readMesh(new File(pathArmadillo, "armadillo.ply")).get
//    val landmarks = LandmarkIO.readLandmarksJson3D(new File(pathArmadillo, "armadillo.json")).get
//    (reference, landmarks)
//  }
//
//  def modelArmadillo(): (PointDistributionModel[_3D, TriangleMesh], Seq[Landmark[_3D]]) = {
//    val model = StatisticalModelIO.readStatisticalTriangleMeshModel3D(new File(pathArmadillo, "armadillo-invlapdot-1000.h5")).get
//    val landmarks = LandmarkIO.readLandmarksJson3D(new File(pathArmadillo, "armadillo.json")).get
//    (model, landmarks)
//  }
//
//  def targetArmadillo(offset: TranslationAfterRotation[_3D] = TranslationAfterRotationSpace3D(Point(0, 0, 0)).identityTransformation)(implicit
//    rnd: Random): (TriangleMesh[_3D], Seq[Landmark[_3D]]) = {
//    val target = MeshIO.readMesh(new File(pathArmadillo, "armadillo_karate.ply")).get.transform(offset)
//    val landmarks = LandmarkIO.readLandmarksJson3D(new File(pathArmadillo, "armadillo_karate.json")).get.map(_.transform(offset))
//    (target, landmarks)
//  }
//
//  def referenceBunny(): (TriangleMesh[_3D], Seq[Landmark[_3D]]) = {
//    val reference = MeshIO.readMesh(new File(pathBunny, "bunny_reference.ply")).get
//    (reference, Seq())
//  }
//
//  def targetBunny(offset: TranslationAfterRotation[_3D] = TranslationAfterRotationSpace3D(Point(0, 0, 0)).identityTransformation)(implicit
//    rnd: Random): (TriangleMesh[_3D], Seq[Landmark[_3D]]) = {
//    val target = MeshIO.readMesh(new File(pathBunny, "target.ply")).get.transform(offset)
//    (target, Seq())
//  }

}
