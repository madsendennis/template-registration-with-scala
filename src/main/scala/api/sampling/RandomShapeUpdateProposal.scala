/*
 * Copyright University of Basel, Graphics and Vision Research Group
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */package api.sampling

import api.GingrRegistrationState
import scalismo.sampling.{ProposalGenerator, TransitionProbability, TransitionRatio}
import scalismo.utils.Random

case class RandomShapeUpdateProposal[State <: GingrRegistrationState[State]](stdev: Double, generatedBy: String = "RandomShapeUpdateProposal")(implicit random: Random)
  extends ProposalGenerator[State] with TransitionRatio[State] with TransitionProbability[State] {

  private val generator = GaussianDenseVectorProposal(stdev)

  override def propose(theta: State): State = {
    println("Random propose")
    val currentCoeffs = theta.general.modelParameters
    val updatedCoeffs = generator.propose(currentCoeffs)
    theta.updateGeneral(theta.general.updateIteration(theta.general.iteration - 1).updateModelParameters(updatedCoeffs))
  }

  override def logTransitionProbability(from: State, to: State): Double = {
    generator.logTransitionProbability(from.general.modelParameters,to.general.modelParameters)
  }
}
