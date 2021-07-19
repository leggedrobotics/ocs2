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

#include "ocs2_ballbot/BallbotInterface.h"

#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>

#include <ros/package.h>

namespace ocs2 {
namespace ballbot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
BallbotInterface::BallbotInterface(const std::string& taskFileFolderName) {
  taskFile_ = ros::package::getPath("ocs2_ballbot") + "/config/" + taskFileFolderName + "/task.info";
  std::cerr << "Loading task file: " << taskFile_ << std::endl;

  libraryFolder_ = ros::package::getPath("ocs2_ballbot") + "/auto_generated";
  std::cerr << "Generated library path: " << libraryFolder_ << std::endl;

  // Default initial condition
  loadData::loadEigenMatrix(taskFile_, "initialState", initialState_);
  std::cerr << "x_init:   " << initialState_.transpose() << std::endl;

  // DDP SQP MPC settings
  ddpSettings_ = ddp::loadSettings(taskFile_, "ddp");
  mpcSettings_ = mpc::loadSettings(taskFile_, "mpc");
  sqpSettings_ = multiple_shooting::loadSettings(taskFile_, "multiple_shooting");

  /*
   * Optimal control problem
   */
  // Cost
  matrix_t Q(STATE_DIM, STATE_DIM);
  matrix_t R(INPUT_DIM, INPUT_DIM);
  matrix_t Qf(STATE_DIM, STATE_DIM);
  loadData::loadEigenMatrix(taskFile_, "Q", Q);
  loadData::loadEigenMatrix(taskFile_, "R", R);
  loadData::loadEigenMatrix(taskFile_, "Q_final", Qf);
  std::cerr << "Q:  \n" << Q << "\n";
  std::cerr << "R:  \n" << R << "\n";
  std::cerr << "Q_final:\n" << Qf << "\n";

  problem_.costPtr->add("cost", std::unique_ptr<StateInputCost>(new QuadraticStateInputCost(Q, R)));
  problem_.finalCostPtr->add("finalCost", std::unique_ptr<StateCost>(new QuadraticStateCost(Qf)));

  // Dynamics
  bool recompileLibraries;  // load the flag to generate library files from taskFile
  ocs2::loadData::loadCppDataType(taskFile_, "ballbot_interface.recompileLibraries", recompileLibraries);
  problem_.dynamicsPtr.reset(new BallbotSystemDynamics(libraryFolder_, recompileLibraries));

  // Rollout
  auto rolloutSettings = rollout::loadSettings(taskFile_, "rollout");
  rolloutPtr_.reset(new TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));

  // Initialization
  ballbotInitializerPtr_.reset(new DefaultInitializer(INPUT_DIM));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<MPC_DDP> BallbotInterface::getMpc() {
  return std::unique_ptr<MPC_DDP>(new MPC_DDP(mpcSettings_, ddpSettings_, *rolloutPtr_, problem_, *ballbotInitializerPtr_));
}

}  // namespace ballbot
}  // namespace ocs2
