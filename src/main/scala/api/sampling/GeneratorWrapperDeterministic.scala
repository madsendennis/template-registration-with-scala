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
import scalismo.sampling.{ProposalGenerator, TransitionProbability}
import scalismo.utils.Random

case class GeneratorWrapperDeterministic[State <: GingrRegistrationState[State]](update: (State,Boolean) => State, generatedBy: String)(implicit rnd: Random) extends ProposalGenerator[State] with TransitionProbability[State] {
  override def propose(current: State): State = {
    println("Propose deterministic")
    update(current, false)
  }
  override def logTransitionProbability(from: State, to: State): Double = {
    0.0
  }
}
