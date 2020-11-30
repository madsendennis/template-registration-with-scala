package apps.Affine.cpd

import java.awt.Color
import java.io.File

import api.registration.{CPDAffine, CPDRigid}
import scalismo.geometry.{EuclideanVector3D, Point3D, _3D}
import scalismo.io.MeshIO
import scalismo.transformations.{RigidTransformation, Rotation3D, RotationAfterScalingAfterTranslation, RotationAfterScalingAfterTranslation3D, Scaling, Scaling3D, SimilarityTransformation, Transformation, Translation, Translation3D, TranslationAfterRotation, TranslationAfterRotation3D, TranslationAfterScalingAfterRotation, TranslationAfterScalingAfterRotation3D, TranslationAfterScalingAfterRotationSpace3D}
import scalismo.ui.api.ScalismoUI

object femur extends App {

  scalismo.initialize()

  val rigidTrans: SimilarityTransformation[_3D] = TranslationAfterScalingAfterRotation3D(Translation3D(EuclideanVector3D(50.0, 20.0, 30.0)), Scaling3D(2), Rotation3D(0.1, 0.0, 0.0, Point3D(0,0,0)))

  val source = MeshIO.readMesh(new File("data/femur0_coarse.stl")).get.transform(rigidTrans)
  val target = MeshIO.readMesh(new File("data/femur1_coarse.stl")).get

  println(s"CPD Femur fun - Source points: ${source.pointSet.numberOfPoints}, target points: ${target.pointSet.numberOfPoints}")

  // lambda and beta both reflect the amount of smoothness regularization
  // beta = width of gaussian filter
  // lambda = trade-off between goodness of fit and regularization
  val cpd = new CPDAffine(target, source, lamdba = 10, beta = 100, w = 0.0) // target, source ... Source is moving!!!

  val finalMesh = cpd.Registration(max_iteration = 1000, tolerance = 0.001)
  val ui = ScalismoUI()
  val showTarget = ui.show(target, "target")
  val showSource = ui.show(source, "source")
  val showFinal = ui.show(finalMesh, "final")
  showTarget.color = Color.GREEN
  showSource.color = Color.RED
}
