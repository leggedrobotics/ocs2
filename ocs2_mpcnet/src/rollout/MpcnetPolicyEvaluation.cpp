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

#include "ocs2_mpcnet/rollout/MpcnetPolicyEvaluation.h"

namespace ocs2 {
namespace mpcnet {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
metrics_ptr_t MpcnetPolicyEvaluation::run(const std::string& policyFilePath, scalar_t timeStep, const SystemObservation& initialObservation,
                                          const ModeSchedule& modeSchedule, const TargetTrajectories& targetTrajectories) {
  // declare metrics pointer
  metrics_ptr_t metricsPtr(new metrics_t);

  // init time and state
  scalar_t time = initialObservation.time;
  vector_t state = initialObservation.state;

  // reset mpc
  mpcPtr_->reset();

  // reset rollout, i.e. reset the internal simulator state (e.g. relevant for RaiSim)
  rolloutPtr_->resetRollout();

  // prepare learned controller
  mpcnetPtr_->loadPolicyModel(policyFilePath);

  // update the reference manager
  referenceManagerPtr_->setModeSchedule(modeSchedule);
  referenceManagerPtr_->setTargetTrajectories(targetTrajectories);

  // run policy evaluation
  int iteration = 0;
  try {
    while (time <= targetTrajectories.timeTrajectory.back()) {
      // run mpc and get solution
      if (!mpcPtr_->run(time, state)) {
        throw std::runtime_error("[MpcnetPolicyEvaluation::run] main routine of MPC returned false.");
      }
      const auto primalSolution = mpcPtr_->getSolverPtr()->primalSolution(mpcPtr_->getSolverPtr()->getFinalTime());

      // incurred quantities
      vector_t input = mpcnetPtr_->computeInput(time, state);
      metricsPtr->incurredHamiltonian += mpcPtr_->getSolverPtr()->getHamiltonian(time, state, input).f * timeStep;

      // forward simulate system with learned controller
      scalar_array_t timeTrajectory;
      size_array_t postEventIndicesStock;
      vector_array_t stateTrajectory;
      vector_array_t inputTrajectory;
      rolloutPtr_->run(primalSolution.timeTrajectory_.front(), primalSolution.stateTrajectory_.front(),
                       primalSolution.timeTrajectory_.front() + timeStep, mpcnetPtr_.get(), primalSolution.modeSchedule_.eventTimes,
                       timeTrajectory, postEventIndicesStock, stateTrajectory, inputTrajectory);

      // update time, state and iteration
      time = timeTrajectory.back();
      state = stateTrajectory.back();
      ++iteration;

      // check if forward simulated system diverged
      if (!mpcnetDefinitionPtr_->validState(state)) {
        throw std::runtime_error("[MpcnetPolicyEvaluation::run] state is not valid.");
      }
    }
  } catch (const std::exception& e) {
    // print error for exceptions
    std::cerr << "[MpcnetPolicyEvaluation::run] a standard exception was caught, with message: " << e.what() << "\n";
    // this policy evaluation run failed, incurred quantities are not reported
    metricsPtr->incurredHamiltonian = std::numeric_limits<scalar_t>::quiet_NaN();
  }

  // report survival time
  metricsPtr->survivalTime = time;

  // return metrics pointer
  return metricsPtr;
}

}  // namespace mpcnet
}  // namespace ocs2
