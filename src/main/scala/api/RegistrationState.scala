package api

import breeze.linalg.DenseVector
import scalismo.geometry.{_3D, Landmark}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.PointDistributionModel
import scalismo.transformations.TranslationAfterRotation

trait RegistrationState[T] {
  def maxIterations(): Int // Maximum number of iterations to take
  def model(): PointDistributionModel[_3D, TriangleMesh] // Prior statistical mesh model
  def modelParameters(): DenseVector[Double] // parameters of the current fitting state in the model
  def modelLandmarks(): Option[Seq[Landmark[_3D]]] // Landmarks on the model
  def target(): TriangleMesh[_3D] // Target mesh
  def targetLandmarks(): Option[Seq[Landmark[_3D]]] // Landmarks on the target
  def fit(): TriangleMesh[_3D] // Current fit based on model parameters, global alignment and scaling
  def alignment(): TranslationAfterRotation[_3D] // Model translation and rotation
  def scaling(): Double // Model scaling
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
  private[api] def updateAlignment(next: TranslationAfterRotation[_3D]): T
  private[api] def updateScaling(next: Double): T
  private[api] def updateModelParameters(next: DenseVector[Double]): T
  private[api] def updateGeneratedBy(next: String): T
}
