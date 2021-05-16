/*
 *  Copyright University of Basel, Graphics and Vision Research Group
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

package api.sampling.evaluators

import api.registration.utils.{ClosestPointRegistrator, ReferenceToTarget, TargetToReference}
import api.sampling.ModelFittingParameters
import breeze.stats.distributions.ContinuousDistr
import scalismo.common.PointId
import scalismo.geometry.{Point, _3D}
import scalismo.mesh.TriangleMesh3D
import scalismo.numerics.UniformMeshSampler3D
import scalismo.sampling.DistributionEvaluator
import scalismo.statisticalmodel.StatisticalMeshModel
import scalismo.utils.Random.implicits._

case class IndependentPointDistanceEvaluatorBoundary(model: StatisticalMeshModel,
                                                     targetMesh: TriangleMesh3D,
                                                     likelihoodModel: ContinuousDistr[Double],
                                                     evaluationMode: EvaluationMode,
                                                     numberOfPointsForComparison: Int,
                                                     boundaryAware: Boolean = false)
  extends DistributionEvaluator[ModelFittingParameters] with EvaluationCaching {

  private val decModel = model.decimate(numberOfPointsForComparison)
  private val decTarget  = targetMesh.operations.decimate(numberOfPointsForComparison)

  // Make sure not to oversample if the numberOfPointsForComparison is set higher than the points in the target or the model
  def distModelToTarget(modelSample: TriangleMesh3D): Double = {
    val corrInit = ClosestPointRegistrator.ClosestPointTriangleMesh3D.closestPointCorrespondence(modelSample, targetMesh, direction = ReferenceToTarget)

    val corr = if(boundaryAware) corrInit._1.filter(_._3 == 1.0) else corrInit._1

    val dists = corr.map{case (id, p, _) =>
      val v = modelSample.pointSet.point(id)-p
      likelihoodModel.logPdf(v.norm)
    }
    dists.sum/corr.length
  }


  def distTargetToModel(modelSample: TriangleMesh3D): Double = {
    val corrInit = ClosestPointRegistrator.ClosestPointTriangleMesh3D.closestPointCorrespondence(modelSample, decTarget, direction = TargetToReference)

    val corr = if(boundaryAware) corrInit._1.filter(_._3 == 1.0) else corrInit._1

    val dists = corr.map{case (id, p, _) =>
      val v = modelSample.pointSet.point(id)-p
      likelihoodModel.logPdf(v.norm)
    }
    dists.sum/corr.length
  }


  def computeLogValue(sample: ModelFittingParameters): Double = {

    val currentSampleFull = ModelFittingParameters.transformedMesh(model, sample)
    val currentSampleDec = ModelFittingParameters.transformedMesh(decModel, sample)

    val dist = evaluationMode match {
      case ModelToTargetEvaluation => distModelToTarget(currentSampleDec)
      case TargetToModelEvaluation => distTargetToModel(currentSampleFull)
      case SymmetricEvaluation => 0.5 * distModelToTarget(currentSampleDec) + 0.5 * distTargetToModel(currentSampleFull)
    }
    dist
  }
}



