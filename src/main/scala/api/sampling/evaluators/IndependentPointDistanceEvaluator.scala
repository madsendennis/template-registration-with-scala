package api.sampling.evaluators

import api.GingrRegistrationState
import breeze.stats.distributions.ContinuousDistr
import scalismo.common.{DomainWarp, PointId}
import scalismo.geometry.{Point, _3D}
import scalismo.mesh.{TriangleMesh, TriangleMesh3D}
import scalismo.sampling.DistributionEvaluator

trait EvaluationMode

case object ModelToTargetEvaluation extends EvaluationMode

case object TargetToModelEvaluation extends EvaluationMode

case object SymmetricEvaluation extends EvaluationMode

case class IndependentPointDistanceEvaluator[State <: GingrRegistrationState[State]]
(
  sample: State,
  likelihoodModel: ContinuousDistr[Double],
  evaluationMode: EvaluationMode,
  numberOfPointsForComparison: Int
)
  extends DistributionEvaluator[State] with EvaluationCaching[State] {

  private val instance = sample.general.fit
  private val instanceDecimated = instance.operations.decimate(numberOfPointsForComparison)

  private val target = sample.general.target
  private val targetDecimated = target.operations.decimate(numberOfPointsForComparison)

  private val randomPointsOnTarget: IndexedSeq[Point[_3D]] = targetDecimated.pointSet.points.toIndexedSeq
  private val randomPointIdsOnModel: IndexedSeq[PointId] = instanceDecimated.pointSet.pointIds.toIndexedSeq

  def distModelToTarget(modelSample: TriangleMesh3D): Double = {
    val pointsOnSample = randomPointIdsOnModel.map(modelSample.pointSet.point)
    val dists = for (pt <- pointsOnSample) yield {
      likelihoodModel.logPdf((target.operations.closestPointOnSurface(pt).point - pt).norm)
    }
    dists.sum
  }


  def distTargetToModel(modelSample: TriangleMesh3D): Double = {
    val dists = for (pt <- randomPointsOnTarget) yield {
      likelihoodModel.logPdf((modelSample.operations.closestPointOnSurface(pt).point - pt).norm)
    }
    dists.sum
  }


  def computeLogValue(sample: State): Double = {

    val currentSample = sample.general.fit
    val dist = evaluationMode match {
      case ModelToTargetEvaluation => distModelToTarget(currentSample)
      case TargetToModelEvaluation => distTargetToModel(currentSample)
      case SymmetricEvaluation => 0.5 * distModelToTarget(currentSample) + 0.5 * distTargetToModel(currentSample)
    }
    dist
  }
}
