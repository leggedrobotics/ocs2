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

#include "ocs2_ballbot_example/BallbotInterface.h"

#include <ocs2_core/misc/LoadData.h>

#include <ros/package.h>

namespace ocs2 {
namespace ballbot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
BallbotInterface::BallbotInterface(const std::string& taskFileFolderName) {
  taskFile_ = ros::package::getPath("ocs2_ballbot_example") + "/config/" + taskFileFolderName + "/task.info";
  std::cerr << "Loading task file: " << taskFile_ << std::endl;

  libraryFolder_ = ros::package::getPath("ocs2_ballbot_example") + "/auto_generated";
  std::cerr << "Generated library path: " << libraryFolder_ << std::endl;

  // load setting from loading file
  loadSettings(taskFile_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void BallbotInterface::loadSettings(const std::string& taskFile) {
  /*
   * Default initial condition
   */
  loadData::loadEigenMatrix(taskFile, "initialState", initialState_);

  /*
   * DDP SQP MPC settings
   */
  sqpSettings_ = multiple_shooting::loadSettings(taskFile, "multiple_shooting");
  ddpSettings_ = ddp::loadSettings(taskFile, "ddp");
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc");

  /*
   * Dynamics
   */
  // load the flag to generate library files from taskFile
  bool recompileLibraries;
  ocs2::loadData::loadCppDataType(taskFile_, "ballbot_interface.recompileLibraries", recompileLibraries);

  ballbotSystemDynamicsPtr_.reset(new BallbotSystemDynamics());
  ballbotSystemDynamicsPtr_->initialize("ballbot_dynamics", libraryFolder_, recompileLibraries, true);

  /*
   * Rollout
   */
  auto rolloutSettings = rollout::loadSettings(taskFile, "rollout");
  ddpBallbotRolloutPtr_.reset(new TimeTriggeredRollout(*ballbotSystemDynamicsPtr_, rolloutSettings));

  /*
   * Cost function
   */
  ocs2::loadData::loadEigenMatrix(taskFile, "Q", Q_);
  ocs2::loadData::loadEigenMatrix(taskFile, "R", R_);
  ocs2::loadData::loadEigenMatrix(taskFile, "Q_final", QFinal_);

  std::cerr << "Q:  \n" << Q_ << std::endl;
  std::cerr << "R:  \n" << R_ << std::endl;
  std::cerr << "Q_final:\n" << QFinal_ << std::endl;
  std::cerr << "x_init:   " << initialState_.transpose() << std::endl;

  ballbotCostPtr_.reset(new QuadraticCostFunction(Q_, R_, QFinal_));

  /*
   * Constraints
   */
  ballbotConstraintPtr_.reset(new ConstraintBase());

  /*
   * Initialization
   */
  ballbotInitializerPtr_.reset(new DefaultInitializer(INPUT_DIM));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<MPC_DDP> BallbotInterface::getMpc() {
  return std::unique_ptr<MPC_DDP>(new MPC_DDP(ddpBallbotRolloutPtr_.get(), ballbotSystemDynamicsPtr_.get(), ballbotConstraintPtr_.get(),
                                              ballbotCostPtr_.get(), ballbotInitializerPtr_.get(), ddpSettings_, mpcSettings_));
}

}  // namespace ballbot
}  // namespace ocs2
