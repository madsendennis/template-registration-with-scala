package api

import breeze.linalg.DenseVector
import scalismo.geometry.{_3D, EuclideanVector, EuclideanVector3D, Landmark, Point}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.PointDistributionModel
import scalismo.transformations.{Rotation, RotationSpace3D, Translation, TranslationAfterRotation}

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

case class ModelFittingParameters(scale: ScaleParameter, pose: PoseParameters, shape: ShapeParameters, generatedBy: String = "Anonymous") {
  val allParameters: DenseVector[Double] = DenseVector.vertcat(scale.parameters, pose.parameters, shape.parameters)

}

trait RegistrationState[T] {
  def maxIterations(): Int // Maximum number of iterations to take
  def model(): PointDistributionModel[_3D, TriangleMesh] // Prior statistical mesh model
  def modelParameters(): ModelFittingParameters // parameters of the current fitting state in the model
  def modelLandmarks(): Option[Seq[Landmark[_3D]]] // Landmarks on the model
  def target(): TriangleMesh[_3D] // Target mesh
  def targetLandmarks(): Option[Seq[Landmark[_3D]]] // Landmarks on the target
  def fit(): TriangleMesh[_3D] // Current fit based on model parameters, global alignment and scaling
//  def translation(): Translation[_3D] // Model translation and rotation
//  def rotation(): Rotation[_3D] // Model translation and rotation
//  def scaling(): Double // Model scaling
  def converged(): Boolean // Has the registration converged???
  def sigma2(): Double // Global uncertainty parameter
  def threshold: Double // Convergence threshold
  def globalTransformation(): GlobalTranformationType // Type of global transformation (none, rigid, similarity)
  def stepLength(): Double // Step length of a single registration step (0.0 to 1.0)
  def generatedBy(): String // Name of generator that produced the State
//  def probabilistic(): Boolean //
//  def nonRigidTransformation(): Boolean

  /** Updates the current state with the new fit.
    *
    * @param next
    *   The newly calculated shape / fit.
    */
  private[api] def updateFit(next: TriangleMesh[_3D]): T
//  private[api] def updateAlignment(next: TranslationAfterRotation[_3D]): T
  private[api] def updateTranslation(next: EuclideanVector[_3D]): T
  private[api] def updateRotation(next: EulerRotation): T
  private[api] def updateRotation(next: Rotation[_3D]): T
  private[api] def updateScaling(next: ScaleParameter): T
  private[api] def updateShapeParameters(next: ShapeParameters): T
  private[api] def updateModelParameters(next: ModelFittingParameters): T
  private[api] def updateGeneratedBy(next: String): T
}
