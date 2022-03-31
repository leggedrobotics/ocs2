/******************************************************************************
Copyright (c) 2022, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "ocs2_mpcnet_core/rollout/MpcnetPolicyEvaluation.h"

namespace ocs2 {
namespace mpcnet {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
metrics_t MpcnetPolicyEvaluation::run(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep,
                                      const SystemObservation& initialObservation, const ModeSchedule& modeSchedule,
                                      const TargetTrajectories& targetTrajectories) {
  // declare metrics
  metrics_t metrics;

  // set system
  set(alpha, policyFilePath, initialObservation, modeSchedule, targetTrajectories);

  // run policy evaluation
  try {
    while (systemObservation_.time <= targetTrajectories.timeTrajectory.back()) {
      // step system
      step(timeStep);

      // incurred quantities
      const scalar_t time = primalSolution_.timeTrajectory_.front();
      const vector_t state = primalSolution_.stateTrajectory_.front();
      const vector_t input = behavioralControllerPtr_->computeInput(time, state);
      metrics.incurredHamiltonian += mpcPtr_->getSolverPtr()->getHamiltonian(time, state, input).f * timeStep;
    }
  } catch (const std::exception& e) {
    // print error for exceptions
    std::cerr << "[MpcnetPolicyEvaluation::run] a standard exception was caught, with message: " << e.what() << "\n";
    // this policy evaluation run failed, incurred quantities are not reported
    metrics.incurredHamiltonian = std::numeric_limits<scalar_t>::quiet_NaN();
  }

  // report survival time
  metrics.survivalTime = systemObservation_.time;

  // return metrics
  return metrics;
}

}  // namespace mpcnet
}  // namespace ocs2
