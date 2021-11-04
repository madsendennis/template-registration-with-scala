package other.api.utils

import scalismo.geometry.{_3D, Landmark, Point}
import scalismo.mesh.TriangleMesh
import scalismo.registration.LandmarkRegistration
import scalismo.statisticalmodel.PointDistributionModel
import scalismo.transformations.TranslationAfterRotation

object AlignmentTransforms {

  def computeTransform(lm1: Seq[Landmark[_3D]], lm2: Seq[Landmark[_3D]], center: Point[_3D]): TranslationAfterRotation[_3D] = {
    val commonLmNames = lm1.map(_.id) intersect lm2.map(_.id)

    val landmarksPairs = commonLmNames.map(name => (lm1.find(_.id == name).get.point, lm2.find(_.id == name).get.point))
    LandmarkRegistration.rigid3DLandmarkRegistration(landmarksPairs, center)
  }

  def alignModelToTarget(
    model: PointDistributionModel[_3D, TriangleMesh],
    modelLandmarks: Seq[Landmark[_3D]],
    targetLandmarks: Seq[Landmark[_3D]]): PointDistributionModel[_3D, TriangleMesh] = {
    val t = computeTransform(modelLandmarks, targetLandmarks, Point(0, 0, 0))
    model.transform(t)
  }

  def alignTargetToModel(target: TriangleMesh[_3D], modelLandmarks: Seq[Landmark[_3D]], targetLandmarks: Seq[Landmark[_3D]]): TriangleMesh[_3D] = {
    val t = computeTransform(targetLandmarks, modelLandmarks, Point(0, 0, 0))
    target.transform(t)
  }
}
