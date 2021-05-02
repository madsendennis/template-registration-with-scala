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

package api.sampling

import api.other.{IcpProjectionDirection, ModelAndTargetSampling, ModelSampling, TargetSampling}
import api.sampling.proposals._
import samplingtools.proposals.{GaussianAxisRotationProposal, GaussianAxisTranslationProposal, PitchAxis, RollAxis, StandardRandomPoseProposal, YawAxis}
import scalismo.geometry.{Landmark, _3D}
import scalismo.mesh.TriangleMesh3D
import scalismo.sampling.proposals.MixtureProposal
import scalismo.sampling.proposals.MixtureProposal.ProposalGeneratorWithTransition
import scalismo.sampling.proposals.MixtureProposal.implicits._
import scalismo.statisticalmodel.StatisticalMeshModel
import scalismo.utils.Random.implicits._

object MixedProposalDistributions {

  def randomPoseProposal(scaling: Double): ProposalGeneratorWithTransition[ModelFittingParameters] ={
    StandardRandomPoseProposal(scaling)
  }

  def myRandomPoseProposal(): ProposalGeneratorWithTransition[ModelFittingParameters] ={
    val mixproposal = MixtureProposal(
      0.5 *: GaussianAxisRotationProposal(0.001, YawAxis, generatedBy = "RotationYaw") +
      0.5 *: GaussianAxisRotationProposal(0.001, PitchAxis, generatedBy = "RotationPitch") +
      0.5 *: GaussianAxisRotationProposal(0.01, RollAxis, generatedBy = "RotationRoll") +
      0.5 *: GaussianAxisTranslationProposal(1.0, 0, generatedBy = "TranslationX") +
      0.5 *: GaussianAxisTranslationProposal(1.0, 0, generatedBy = "TranslationY") +
      0.5 *: GaussianAxisTranslationProposal(1.0, 0, generatedBy = "TranslationZ")
    )
    mixproposal
  }

  def mixedProposalRandom(model: StatisticalMeshModel): ProposalGeneratorWithTransition[ModelFittingParameters] = {
    val mixproposal = MixtureProposal(
        0.5 *: RandomShapeUpdateProposal(model, 0.1, generatedBy = "RandomShape-0.1") //+
//        0.5 *: RandomShapeUpdateProposal(model, 0.01, generatedBy = "RandomShape-0.01") +
//        0.5 *: RandomShapeUpdateProposal(model, 0.001, generatedBy = "RandomShape-0.001")
    )
    mixproposal
  }

  def mixedProposalICP(model: StatisticalMeshModel, target: TriangleMesh3D, modelLM: Seq[Landmark[_3D]], targetLM: Seq[Landmark[_3D]],numOfSamplePoints: Int, projectionDirection: IcpProjectionDirection = ModelAndTargetSampling, tangentialNoise: Double = 100.0, noiseAlongNormal: Double = 3.0, stepLength: Double = 0.1, boundaryAware: Boolean = true): ProposalGeneratorWithTransition[ModelFittingParameters] = {

    val rate = 0.5

    val modelSamplingProposals: Seq[(Double, NonRigidIcpProposal)] = Seq((rate, NonRigidIcpProposal(model, target, modelLM, targetLM, stepLength, tangentialNoise = tangentialNoise, noiseAlongNormal = noiseAlongNormal, projectionDirection = ModelSampling, boundaryAware, generatedBy = s"IcpProposal-ModelSampling-${stepLength}Step")))

    val targetSamplingProposals: Seq[(Double, NonRigidIcpProposal)] = Seq((rate, NonRigidIcpProposal(model, target, modelLM, targetLM, stepLength, tangentialNoise = tangentialNoise, noiseAlongNormal = noiseAlongNormal, projectionDirection = TargetSampling, boundaryAware, generatedBy = s"IcpProposal-TargetSampling-${stepLength}Step")))

    def proposals: Seq[(Double, NonRigidIcpProposal)] = {
      if (projectionDirection == TargetSampling) {
        targetSamplingProposals
      } else if (projectionDirection == ModelSampling) {
        modelSamplingProposals
      }
      else {
        targetSamplingProposals ++ modelSamplingProposals
      }
    }

    MixtureProposal.fromProposalsWithTransition(proposals: _ *)
  }

}
