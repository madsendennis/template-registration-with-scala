package api

import breeze.linalg.DenseVector
import scalismo.geometry.{_3D, EuclideanVector, Point}
import scalismo.transformations.{Rotation, Scaling, Translation, TranslationAfterRotation, TranslationAfterScalingAfterRotation}

case class ScaleParameter(s: Double) {
  def parameters: DenseVector[Double] = DenseVector(s)
}

case class EulerAngles(phi: Double, theta: Double, psi: Double) {
  def parameters: DenseVector[Double] = DenseVector(phi, theta, psi)
}

case class EulerRotation(angles: EulerAngles, center: Point[_3D]) {
  val rotation: Rotation[_3D] = Rotation(angles.phi, angles.theta, angles.psi, center)
  def parameters: DenseVector[Double] = DenseVector.vertcat(angles.parameters, center.toBreezeVector)
}

case class PoseParameters(translation: EuclideanVector[_3D], rotation: EulerRotation) {
  def parameters: DenseVector[Double] = {
    DenseVector.vertcat(
      DenseVector.vertcat(
        translation.toBreezeVector,
        rotation.parameters
      )
    )
  }
}

case class ShapeParameters(parameters: DenseVector[Double])

case class ModelFittingParameters(scale: ScaleParameter, pose: PoseParameters, shape: ShapeParameters) {
  val allParameters: DenseVector[Double] = DenseVector.vertcat(scale.parameters, pose.parameters, shape.parameters)

  def rigidTransform: TranslationAfterRotation[_3D] = {
    val translation = Translation(pose.translation)
    val rotation = pose.rotation.rotation
    TranslationAfterRotation(translation, rotation)
  }

  def similarityTransform: TranslationAfterScalingAfterRotation[_3D] = {
    val translation = Translation(pose.translation)
    val rotation = pose.rotation.rotation
    val scaling = Scaling[_3D](scale.s)
    TranslationAfterScalingAfterRotation(translation, scaling, rotation)
  }
}
