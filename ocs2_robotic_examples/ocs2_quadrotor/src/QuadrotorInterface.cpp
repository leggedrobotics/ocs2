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

#include "ocs2_quadrotor/QuadrotorInterface.h"
#include "ocs2_quadrotor/dynamics/QuadrotorSystemDynamics.h"

#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/initialization/OperatingPoints.h>
#include <ocs2_core/misc/LoadData.h>

#include <ros/package.h>

namespace ocs2 {
namespace quadrotor {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QuadrotorInterface::QuadrotorInterface(const std::string& taskFileFolderName) {
  taskFile_ = ros::package::getPath("ocs2_quadrotor") + "/config/" + taskFileFolderName + "/task.info";
  std::cerr << "Loading task file: " << taskFile_ << std::endl;

  libraryFolder_ = ros::package::getPath("ocs2_quadrotor") + "/auto_generated";
  std::cerr << "Generated library path: " << libraryFolder_ << std::endl;

  // load setting from loading file
  loadSettings(taskFile_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadrotorInterface::loadSettings(const std::string& taskFile) {
  /*
   * Default initial condition
   */
  loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
  std::cerr << "x_init:   " << initialState_.transpose() << std::endl;

  /*
   * Solver settings
   */
  ddpSettings_ = ddp::loadSettings(taskFile, "ddp");
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc");

  /*
   * quadrotor parameters
   */
  auto quadrotorParameters = quadrotor::loadSettings(taskFile, "QuadrotorParameters", true);

  /*
   * Dynamics and derivatives
   */
  std::unique_ptr<QuadrotorSystemDynamics> dynamicsPtr(new QuadrotorSystemDynamics(quadrotorParameters));

  /*
   * Rollout
   */
  auto rolloutSettings = rollout::loadSettings(taskFile, "rollout");
  rolloutPtr_.reset(new TimeTriggeredRollout(*dynamicsPtr, rolloutSettings));

  /*
   * Optimal control problem
   */
  problem_.dynamicsPtr = std::move(dynamicsPtr);

  /*
   * Cost function
   */
  matrix_t Q(STATE_DIM, STATE_DIM);
  matrix_t R(INPUT_DIM, INPUT_DIM);
  matrix_t Qf(STATE_DIM, STATE_DIM);
  loadData::loadEigenMatrix(taskFile, "Q", Q);
  loadData::loadEigenMatrix(taskFile, "R", R);
  loadData::loadEigenMatrix(taskFile, "Q_final", Qf);
  std::cerr << "Q:  \n" << Q << std::endl;
  std::cerr << "R:  \n" << Q << std::endl;
  std::cerr << "Q_final:\n" << Qf << std::endl;

  problem_.costPtr->add("cost", std::unique_ptr<StateInputCost>(new QuadraticStateInputCost(Q, R)));
  problem_.finalCostPtr->add("finalCost", std::unique_ptr<StateCost>(new QuadraticStateCost(Qf)));

  /*
   * Initialization
   */
  vector_t initialInput = vector_t::Zero(INPUT_DIM);
  initialInput(0) = quadrotorParameters.quadrotorMass_ * quadrotorParameters.gravity_;
  operatingPointPtr_.reset(new OperatingPoints(initialState_, initialInput));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<MPC_DDP> QuadrotorInterface::getMpc() {
  return std::unique_ptr<MPC_DDP>(new MPC_DDP(mpcSettings_, ddpSettings_, *rolloutPtr_, problem_, *operatingPointPtr_));
}

}  // namespace quadrotor
}  // namespace ocs2
