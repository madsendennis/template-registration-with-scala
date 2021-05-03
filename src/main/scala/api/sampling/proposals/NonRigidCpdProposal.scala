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

package api.sampling.proposals

import api.other.{IcpProjectionDirection, ModelSampling, TargetSampling}
import api.registration.cpd.NonRigidCPDwithGPMM
import api.registration.utils.{ClosestPointRegistrator, PointSequenceConverter, ReferenceToTarget, TargetToReference}
import api.sampling.{ModelFittingParameters, ShapeParameters}
import breeze.linalg.{Axis, DenseMatrix, sum}
import scalismo.common.{Field, PointId}
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.geometry._
import scalismo.mesh.{TriangleMesh, TriangleMesh3D}
import scalismo.sampling.{ProposalGenerator, TransitionProbability}
import scalismo.statisticalmodel.{LowRankGaussianProcess, MultivariateNormalDistribution, PointDistributionModel, StatisticalMeshModel}
import scalismo.transformations.RigidTransformation
import scalismo.utils.Memoize

case class NonRigidCpdProposal(
                                model: StatisticalMeshModel,
                                target: TriangleMesh3D,
                                stepLength: Double,
                                generatedBy: String = "ShapeCpdProposal"
                              )(
                                implicit rand: scalismo.utils.Random
                              ) extends ProposalGenerator[ModelFittingParameters]
  with TransitionProbability[ModelFittingParameters] {

  private val pdm = PointDistributionModel[_3D, TriangleMesh](model.referenceMesh, model.gp.interpolate(NearestNeighborInterpolator()))
  private val cpd = new NonRigidCPDwithGPMM[_3D, TriangleMesh](pdm, target, Seq(), Seq(), lambda = 1, w=0, 100, None)
  private val refPoints = pdm.reference.pointSet.points.toSeq
  private val targetPoints = target.pointSet.points.toSeq
  var sigma2: Double = cpd.computeInitialSigma2(refPoints, targetPoints)
//  private val X = PointSequenceConverter.denseMatrixToPoint3DSequence.toMatrix(targetPoints)
//  println(s"Init sigma2: ${sigma2}")
//  private val referenceMesh = model.referenceMesh
  private val cashedPosterior: Memoize[ModelFittingParameters, (StatisticalMeshModel, DenseMatrix[Double])] = Memoize(cpdPosterior, 20)

//  private lazy val interpolatedModel = model.gp.interpolate(NearestNeighborInterpolator())

  override def propose(theta: ModelFittingParameters): ModelFittingParameters = {
    val posInfo = cashedPosterior(theta)
    val posterior = posInfo._1
    val pmat = posInfo._2
    val proposed = posterior.sample()

    val newCoefficients = pdm.coefficients(proposed)

    val currentShapeCoefficients = theta.shapeParameters.parameters
    val newShapeCoefficients = currentShapeCoefficients + (newCoefficients - currentShapeCoefficients) * stepLength

    val newShape = posterior.instance(newShapeCoefficients)

    val TY = PointSequenceConverter.denseMatrixToPoint3DSequence.toMatrix(newShape.pointSet.points.toSeq)
    val X = PointSequenceConverter.denseMatrixToPoint3DSequence.toMatrix(targetPoints)
    sigma2 = cpd.computeSigma2(TY, X, pmat)
//    println(s"Sigma2: ${sigma2}")

    theta.copy(
      shapeParameters = ShapeParameters(newShapeCoefficients),
      generatedBy = generatedBy
    )
  }

  override def logTransitionProbability(from: ModelFittingParameters, to: ModelFittingParameters): Double = {
    val pos = cashedPosterior(from)._1

    val compensatedTo = from.shapeParameters.parameters + ((to.shapeParameters.parameters - from.shapeParameters.parameters) / stepLength)
    val toMesh = model.instance(compensatedTo)

    val projectedTo = pos.coefficients(toMesh)
    val logStuff = pos.gp.logpdf(projectedTo)

    logStuff
}


  private def cpdPosterior(theta: ModelFittingParameters): (StatisticalMeshModel, DenseMatrix[Double]) = {
    val currentMesh = pdm.instance(theta.shapeParameters.parameters)

    val (closestPoints, pmat: DenseMatrix[Double]) = cpd.getCorrespondence(currentMesh.pointSet.points.toSeq, target.pointSet.points.toSeq, sigma2)

    val corr: IndexedSeq[(PointId, Point[_3D], MultivariateNormalDistribution)] = currentMesh.pointSet.pointIds.toSeq.zip(closestPoints).map(t => (t._1, t._2._1, t._2._2)).toIndexedSeq

    (model.posterior(corr), pmat)
  }
}