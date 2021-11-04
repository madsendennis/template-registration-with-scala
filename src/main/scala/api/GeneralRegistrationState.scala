package api

import breeze.linalg.{DenseMatrix, DenseVector}
import scalismo.common.PointId
import scalismo.geometry.{_3D, Landmark, Point}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.{MultivariateNormalDistribution, PointDistributionModel}
import scalismo.transformations._

case class GeneralRegistrationState(
  override val model: PointDistributionModel[_3D, TriangleMesh],
  override val modelParameters: DenseVector[Double],
  override val modelLandmarks: Option[Seq[Landmark[_3D]]] = None,
  override val target: TriangleMesh[_3D],
  override val targetLandmarks: Option[Seq[Landmark[_3D]]] = None,
  override val fit: TriangleMesh[_3D],
  override val alignment: TranslationAfterRotation[_3D],
  override val scaling: Double = 1.0,
  override val converged: Boolean = false,
  override val sigma2: Double = 1.0,
  override val threshold: Double = 1e-10,
  override val maxIterations: Int = 0,
  override val globalTransformation: GlobalTranformationType = RigidTransforms,
  override val stepLength: Double = 1.0,
  override val generatedBy: String = ""
  //  override val probabilistic: Boolean = false
  //  override val nonRigidTransformation: Boolean = true
) extends RegistrationState[GeneralRegistrationState] {

  /** Updates the current state with the new fit.
    *
    * @param next
    *   The newly calculated shape / fit.
    */
  override def updateFit(next: TriangleMesh[_3D]): GeneralRegistrationState = this.copy(fit = next)
  override private[api] def updateAlignment(next: TranslationAfterRotation[_3D]): GeneralRegistrationState = this.copy(alignment = next)
  override private[api] def updateScaling(next: Double): GeneralRegistrationState = this.copy(scaling = next)
  override private[api] def updateModelParameters(next: DenseVector[Double]): GeneralRegistrationState = this.copy(modelParameters = next)
  override private[api] def updateGeneratedBy(next: String): GeneralRegistrationState = this.copy(generatedBy = next)

  lazy val landmarkCorrespondences: IndexedSeq[(PointId, Point[_3D], MultivariateNormalDistribution)] = {
    if (modelLandmarks.nonEmpty && targetLandmarks.nonEmpty) {
      val m = modelLandmarks.get
      val t = targetLandmarks.get
      val commonLmNames = m.map(_.id) intersect t.map(_.id)
      commonLmNames.map { name =>
        val mPoint = m.find(_.id == name).get
        val tPoint = t.find(_.id == name).get
        (
          model.reference.pointSet.findClosestPoint(mPoint.point).id,
          tPoint.point,
          mPoint.uncertainty.getOrElse(MultivariateNormalDistribution(DenseVector.zeros[Double](3), DenseMatrix.eye[Double](3)))
        )
      }.toIndexedSeq
    } else {
      IndexedSeq()
    }
  }
}

object GeneralRegistrationState {
//  def apply(reference: TriangleMesh[_3D], target: TriangleMesh[_3D]): GeneralRegistrationState = {
//    val model: PointDistributionModel[_3D, TriangleMesh] = ???
//    apply(model, target, RigidTransforms)
//  }

  def apply(model: PointDistributionModel[_3D, TriangleMesh], target: TriangleMesh[_3D]): GeneralRegistrationState = {
    apply(model, target, RigidTransforms)
  }

  def apply(model: PointDistributionModel[_3D, TriangleMesh], target: TriangleMesh[_3D], transform: GlobalTranformationType): GeneralRegistrationState = {
    apply(model, Seq(), target, Seq(), transform)
  }

  def apply(
    model: PointDistributionModel[_3D, TriangleMesh],
    modelLandmarks: Seq[Landmark[_3D]],
    target: TriangleMesh[_3D],
    targetLandmarks: Seq[Landmark[_3D]]
  ): GeneralRegistrationState = {
    apply(model, modelLandmarks, target, targetLandmarks, RigidTransforms)
  }

  def apply(
    model: PointDistributionModel[_3D, TriangleMesh],
    modelLandmarks: Seq[Landmark[_3D]],
    target: TriangleMesh[_3D],
    targetLandmarks: Seq[Landmark[_3D]],
    transform: GlobalTranformationType
//    nonRigidTransformation: Boolean
  ): GeneralRegistrationState = {
    val initial =
      new GeneralRegistrationState(
        model = model,
        modelParameters = DenseVector.zeros[Double](model.rank),
        modelLandmarks = Option(modelLandmarks),
        target = target,
        targetLandmarks = Option(targetLandmarks),
        fit = model.mean,
        alignment = TranslationAfterRotationSpace3D(Point(0, 0, 0)).identityTransformation,
        globalTransformation = transform
      )
    initial
  }
}
