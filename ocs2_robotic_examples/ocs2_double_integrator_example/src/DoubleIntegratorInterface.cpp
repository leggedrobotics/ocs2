/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include "ocs2_double_integrator_example/DoubleIntegratorInterface.h"
#include "ocs2_double_integrator_example/dynamics/DoubleIntegratorDynamics.h"

#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/misc/LoadData.h>

#include <ros/package.h>

namespace ocs2 {
namespace double_integrator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
DoubleIntegratorInterface::DoubleIntegratorInterface(const std::string& taskFileFolderName, bool verbose) {
  taskFile_ = ros::package::getPath("ocs2_double_integrator_example") + "/config/" + taskFileFolderName + "/task.info";
  std::cerr << "Loading task file: " << taskFile_ << std::endl;

  libraryFolder_ = ros::package::getPath("ocs2_double_integrator_example") + "/auto_generated";
  std::cerr << "Generated library path: " << libraryFolder_ << std::endl;

  // load setting from loading file
  loadSettings(taskFile_, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void DoubleIntegratorInterface::loadSettings(const std::string& taskFile, bool verbose) {
  /*
   * Default initial condition and final goal
   */
  loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
  loadData::loadEigenMatrix(taskFile, "finalGoal", finalGoal_);

  /*
   * DDP-MPC settings
   */
  ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose);
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose);

  /*
   * Dynamics
   */
  const matrix_t A = (matrix_t(STATE_DIM, STATE_DIM) << 0.0, 1.0, 0.0, 0.0).finished();
  const matrix_t B = (matrix_t(STATE_DIM, INPUT_DIM) << 0.0, 1.0).finished();
  std::unique_ptr<DoubleIntegratorDynamics> dynamicsPtr(new DoubleIntegratorDynamics(A, B));

  /*
   * Rollout
   */
  auto rolloutSettings = rollout::loadSettings(taskFile, "rollout", verbose);
  rolloutPtr_.reset(new TimeTriggeredRollout(*dynamicsPtr, rolloutSettings));

  /*
   * Optimal control problem
   */
  problemPtr_.reset(new OptimalControlProblem);
  problemPtr_->dynamicsPtr = std::move(dynamicsPtr);

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

  problemPtr_->costPtr->add("cost", std::unique_ptr<StateInputCost>(new QuadraticStateInputCost(Q, R)));
  problemPtr_->finalCostPtr->add("finalCost", std::unique_ptr<StateCost>(new QuadraticStateCost(Qf)));

  /*
   * Initialization
   */
  linearSystemInitializerPtr_.reset(new DefaultInitializer(INPUT_DIM));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<MPC_DDP> DoubleIntegratorInterface::getMpc(bool warmStart) {
  if (warmStart) {
    return std::unique_ptr<MPC_DDP>(new MPC_DDP(mpcSettings_, ddpSettings_, *rolloutPtr_, *problemPtr_, *linearSystemInitializerPtr_));

  } else {
    auto mpcSettings = mpcSettings_;
    mpcSettings.coldStart_ = true;
    mpcSettings.runtimeMaxNumIterations_ = mpcSettings.initMaxNumIterations_;
    mpcSettings.runtimeMinStepLength_ = mpcSettings.initMinStepLength_;
    mpcSettings.runtimeMaxStepLength_ = mpcSettings.initMaxStepLength_;
    return std::unique_ptr<MPC_DDP>(new MPC_DDP(mpcSettings_, ddpSettings_, *rolloutPtr_, *problemPtr_, *linearSystemInitializerPtr_));
  }
}

}  // namespace double_integrator
}  // namespace ocs2
