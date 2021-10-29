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
 */
package api

import scalismo.sampling.algorithms.MetropolisHastings
import scalismo.sampling.loggers.{BestSampleLogger, ChainStateLogger, ChainStateLoggerContainer}
import scalismo.sampling.loggers.ChainStateLogger.implicits._
import scalismo.utils.Random

abstract class P_GiNGR[State <: GingrRegistrationState[State]] extends GingrAlgorithm[State] {

  def run(initialState: State, callBack: ChainStateLogger[State])(implicit rng: Random): State = {
    val generator = ???
    val evaluator = ???
    val bestSampleLogger = BestSampleLogger[State](evaluator)
    val loggers = ChainStateLoggerContainer(Seq(bestSampleLogger,callBack))
    val mhChain = MetropolisHastings[State](generator, evaluator)

    ChainStateLogger

    // we need to explicity calculate the states to run the mh-chain
    val states: IndexedSeq[State] = mhChain.
      iterator(initialState).
      loggedWith(loggers).
      take(initialState.general.iteration).
      toIndexedSeq

    val fit = bestSampleLogger.currentBestSample().get
    fit
  }

}
