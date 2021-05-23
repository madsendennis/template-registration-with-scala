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
import api.registration.utils.{ClosestPointRegistrator, ReferenceToTarget, TargetToReference}
import api.sampling.{ModelFittingParameters, ShapeParameters, SurfaceNoiseHelpers}
import breeze.linalg.{DenseMatrix, DenseVector}
import scalismo.common.Field
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.geometry._
import scalismo.mesh.{TriangleMesh, TriangleMesh3D}
import scalismo.sampling.{ProposalGenerator, TransitionProbability}
import scalismo.statisticalmodel.{LowRankGaussianProcess, MultivariateNormalDistribution, StatisticalMeshModel}
import scalismo.transformations.RigidTransformation
import scalismo.utils.Memoize

case class NonRigidProjectionProposal(
                                model: StatisticalMeshModel,
                                target: TriangleMesh3D,
                                modelLM: Seq[Landmark[_3D]],
                                targetLM: Seq[Landmark[_3D]],
                                stepLength: Double,
                                tangentialNoise: Double,
                                noiseAlongNormal: Double,
                                projectionDirection: IcpProjectionDirection = ModelSampling,
                                boundaryAware: Boolean = true,
                                generatedBy: String = "ShapeIcpProposal"
                              )(
                                implicit rand: scalismo.utils.Random
                              ) extends ProposalGenerator[ModelFittingParameters]
  with TransitionProbability[ModelFittingParameters] {

  private val commonLmNames = modelLM.map(_.id) intersect targetLM.map(_.id)

  private val lms = commonLmNames.map{id =>
    val mLM = modelLM.find(_.id == id).get
    val tLM = targetLM.find(_.id == id).get
    val pId = model.referenceMesh.pointSet.findClosestPoint(mLM.point).id
    (pId, tLM.point, mLM.uncertainty.get)
  }.toIndexedSeq


  private val referenceMesh = model.referenceMesh
  private val cashedPosterior: Memoize[ModelFittingParameters, LowRankGaussianProcess[_3D, EuclideanVector[_3D]]] = Memoize(icpPosterior, 20)

  private lazy val interpolatedModel = model.gp.interpolate(NearestNeighborInterpolator())


  override def propose(theta: ModelFittingParameters): ModelFittingParameters = {
    val posterior = cashedPosterior(theta)
    val proposed: Field[_3D, EuclideanVector[_3D]] = posterior.sample()

    def f(pt: Point[_3D]): Point[_3D] = pt + proposed(pt)

    val newCoefficients = model.coefficients(referenceMesh.transform(f))

    val currentShapeCoefficients = theta.shapeParameters.parameters
    val newShapeCoefficients = currentShapeCoefficients + (newCoefficients - currentShapeCoefficients) * stepLength

    theta.copy(
      shapeParameters = ShapeParameters(newShapeCoefficients),
      generatedBy = generatedBy
    )
  }

// Wrong implementation
//  override def logTransitionProbability(from: ModelFittingParameters, to: ModelFittingParameters): Double = {
//    val posterior = cashedPosterior(from)
//    val compensatedTo = from.shapeParameters.parameters + (to.shapeParameters.parameters - from.shapeParameters.parameters) / stepLength
//    val pdf = posterior.logpdf(compensatedTo)
//    pdf
//  }

  override def logTransitionProbability(from: ModelFittingParameters, to: ModelFittingParameters): Double = {
    val pos = cashedPosterior(from)
    val posterior = StatisticalMeshModel(referenceMesh, pos)

    val compensatedTo = from.shapeParameters.parameters + ((to.shapeParameters.parameters - from.shapeParameters.parameters) / stepLength)
    val toMesh = model.instance(compensatedTo)

    val projectedTo = posterior.coefficients(toMesh)
    val logStuff = pos.logpdf(projectedTo)

//    println(s"Wrong log: ${pos.logpdf(compensatedTo)}, correct: ${logStuff}")
    logStuff
    0.0
  }


  private def icpPosterior(theta: ModelFittingParameters): LowRankGaussianProcess[_3D, EuclideanVector[_3D]] = {
    def modelBasedClosestPointsEstimation(
                                           currentMesh: TriangleMesh[_3D],
                                           inversePoseTransform: RigidTransformation[_3D]
                                         ): IndexedSeq[(Point[_3D], EuclideanVector[_3D], MultivariateNormalDistribution)] = {

      val cpInfo = ClosestPointRegistrator.ClosestPointTriangleMesh3D.closestPointCorrespondence(currentMesh, target, direction = ReferenceToTarget)
      val cp = cpInfo._1.filter(_._3==1.0).toIndexedSeq
      val corr = cp.map{case(id, _, _) =>
//        val noiseDistribution = SurfaceNoiseHelpers.surfaceNormalDependantNoise(currentMesh.vertexNormals.atPoint(id), noiseAlongNormal, tangentialNoise)
        val noiseDistribution = MultivariateNormalDistribution(DenseVector.zeros[Double](3), DenseMatrix.eye[Double](3)*0.1)
        val p = currentMesh.pointSet.point(id)
        (id, p, noiseDistribution)
      }

      for ((pointId, targetPoint, uncertainty) <- corr++lms) yield {
        val referencePoint = model.referenceMesh.pointSet.point(pointId)
        // (reference point, deformation vector in model space starting from reference, usually zero-mean observation uncertainty)
        val deformation = referencePoint - referencePoint
        val defNorm = deformation.norm
        (referencePoint, deformation, uncertainty)
      }
    }

    def targetBasedClosestPointsEstimation(
                                            currentMesh: TriangleMesh[_3D],
                                            inversePoseTransform: RigidTransformation[_3D]
                                          ): IndexedSeq[(Point[_3D], EuclideanVector[_3D], MultivariateNormalDistribution)] = {

      val cpInfo = ClosestPointRegistrator.ClosestPointTriangleMesh3D.closestPointCorrespondence(currentMesh, target, direction = TargetToReference)
      val cp = cpInfo._1.filter(_._3==1.0).toIndexedSeq
      val corr = cp.map{case(id, _, _) =>
//        val noiseDistribution = SurfaceNoiseHelpers.surfaceNormalDependantNoise(currentMesh.vertexNormals.atPoint(id), noiseAlongNormal, tangentialNoise)
        val noiseDistribution = MultivariateNormalDistribution(DenseVector.zeros[Double](3), DenseMatrix.eye[Double](3)*0.1)
        val p = currentMesh.pointSet.point(id)
        (id, p, noiseDistribution)
      }

      for ((pointId, targetPoint, uncertainty) <- corr++lms) yield {
        val referencePoint = model.referenceMesh.pointSet.point(pointId)
        // (reference point, deformation vector in model space starting from reference, usually zero-mean observation uncertainty)
        val deformation = referencePoint - referencePoint
        val defNorm = deformation.norm
        (referencePoint, deformation, uncertainty)
      }
    }

    /**
      * Estimate where points should move to together with a surface normal dependant noise.
      *
      * @param theta Current fitting parameters
      * @return List of points, with associated deformation and normal dependant surface noise.
      */
    def uncertainDisplacementEstimation(theta: ModelFittingParameters)
    : IndexedSeq[(Point[_3D], EuclideanVector[_3D], MultivariateNormalDistribution)] = {
      val currentMesh = ModelFittingParameters.transformedMesh(model, theta)
      val inversePoseTransform = ModelFittingParameters.poseTransform(theta).inverse

      if (projectionDirection == TargetSampling) {
        targetBasedClosestPointsEstimation(currentMesh, inversePoseTransform)
      }
      else {
        modelBasedClosestPointsEstimation(currentMesh, inversePoseTransform)
      }
    }

    val uncertainDisplacements = uncertainDisplacementEstimation(theta)
    interpolatedModel.posterior(uncertainDisplacements)
  }

}