/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include "ocs2_mpc/MPC_MRT_Interface.h"

#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MPC_MRT_Interface::MPC_MRT_Interface(MPC_BASE& mpc) : MRT_BASE(), mpc_(mpc), costDesiredTrajectoriesBufferUpdated_(false) {
  mpcTimer_.reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_MRT_Interface::resetMpcNode(const CostDesiredTrajectories& initCostDesiredTrajectories) {
  mpc_.reset();
  mpcTimer_.reset();
  setTargetTrajectories(initCostDesiredTrajectories);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_MRT_Interface::setTargetTrajectories(const CostDesiredTrajectories& targetTrajectories) {
  std::lock_guard<std::mutex> lock(costDesiredTrajectoriesBufferMutex_);
  costDesiredTrajectoriesBuffer_ = targetTrajectories;
  costDesiredTrajectoriesBufferUpdated_ = true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_MRT_Interface::setCurrentObservation(const SystemObservation& currentObservation) {
  std::lock_guard<std::mutex> lock(observationMutex_);
  currentObservation_ = currentObservation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_MRT_Interface::advanceMpc() {
  // measure the delay in running MPC
  mpcTimer_.startTimer();

  SystemObservation currentObservation;
  {
    std::lock_guard<std::mutex> lock(observationMutex_);
    currentObservation = currentObservation_;
  }

  // Set latest cost desired trajectories
  if (costDesiredTrajectoriesBufferUpdated_) {
    std::lock_guard<std::mutex> lock(costDesiredTrajectoriesBufferMutex_);
    mpc_.getSolverPtr()->swapCostDesiredTrajectories(costDesiredTrajectoriesBuffer_);
    costDesiredTrajectoriesBufferUpdated_ = false;

    if (mpc_.settings().debugPrint_) {
      std::cerr << "### The target position is updated to\n";
      mpc_.getSolverPtr()->getCostDesiredTrajectories().display();
    }
  }

  bool controllerIsUpdated = mpc_.run(currentObservation.time, currentObservation.state);
  if (!controllerIsUpdated) {
    return;
  }
  fillMpcOutputBuffers(currentObservation);

  // measure the delay for sending ROS messages
  mpcTimer_.endTimer();

  // check MPC delay and solution window compatibility
  scalar_t timeWindow = mpc_.settings().solutionTimeWindow_;
  if (mpc_.settings().solutionTimeWindow_ < 0) {
    timeWindow = mpc_.getSolverPtr()->getFinalTime() - currentObservation.time;
  }
  if (timeWindow < 2.0 * mpcTimer_.getAverageInMilliseconds() * 1e-3) {
    std::cerr << "WARNING: The solution time window might be shorter than the MPC delay!" << std::endl;
  }

  // measure the delay
  if (mpc_.settings().debugPrint_) {
    std::cerr << "\n### MPC_MRT Benchmarking";
    std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms].";
    std::cerr << "\n###   Latest  : " << mpcTimer_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_MRT_Interface::fillMpcOutputBuffers(SystemObservation mpcInitObservation) {
  // get new policy from solver
  auto newSolution = std::unique_ptr<PrimalSolution>(new PrimalSolution);
  const scalar_t startTime = mpcInitObservation.time;
  const scalar_t finalTime =
      (mpc_.settings().solutionTimeWindow_ < 0) ? mpc_.getSolverPtr()->getFinalTime() : startTime + mpc_.settings().solutionTimeWindow_;
  mpc_.getSolverPtr()->getPrimalSolution(finalTime, newSolution.get());

  // get new command from solver
  auto newCommand = std::unique_ptr<CommandData>(new CommandData);
  newCommand->mpcInitObservation_ = std::move(mpcInitObservation);
  newCommand->mpcCostDesiredTrajectories_ = mpc_.getSolverPtr()->getCostDesiredTrajectories();

  this->fillSolutionBuffer(std::move(newCommand), std::move(newSolution));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_MRT_Interface::getLinearFeedbackGain(scalar_t time, matrix_t& K) {
  auto controller = dynamic_cast<LinearController*>(this->getPolicy().controllerPtr_.get());
  if (controller == nullptr) {
    throw std::runtime_error("Feedback gains only available with linear controller");
  }
  controller->getFeedbackGain(time, K);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t MPC_MRT_Interface::getValueFunction(scalar_t time, const vector_t& state) {
  return mpc_.getSolverPtr()->getValueFunction(time, state);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_MRT_Interface::getValueFunctionStateDerivative(scalar_t time, const vector_t& state, vector_t& Vx) {
  Vx = mpc_.getSolverPtr()->getValueFunctionStateDerivative(time, state);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_MRT_Interface::getStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t& state, vector_t& nu) const {
  mpc_.getSolverPtr()->getStateInputEqualityConstraintLagrangian(time, state, nu);
}

}  // namespace ocs2
