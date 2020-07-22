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

#include "ocs2_cart_pole_example/CartPoleInterface.h"
#include <ros/package.h>

namespace ocs2 {
namespace cartpole {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CartPoleInterface::CartPoleInterface(const std::string& taskFileFolderName)
    : qm_(STATE_DIM_, STATE_DIM_),
      rm_(INPUT_DIM_, INPUT_DIM_),
      qmFinal_(STATE_DIM_, STATE_DIM_),
      xFinal_(STATE_DIM_),
      xNominal_(STATE_DIM_),
      uNominal_(INPUT_DIM_),
      initialState_(STATE_DIM_) {
  taskFile_ = ros::package::getPath("ocs2_cart_pole_example") + "/config/" + taskFileFolderName + "/task.info";
  std::cerr << "Loading task file: " << taskFile_ << std::endl;

  libraryFolder_ = ros::package::getPath("ocs2_cart_pole_example") + "/auto_generated";
  std::cerr << "Generated library path: " << libraryFolder_ << std::endl;

  // load setting from loading file
  loadSettings(taskFile_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CartPoleInterface::loadSettings(const std::string& taskFile) {
  /*
   * Default initial condition
   */
  loadData::loadEigenMatrix(taskFile, "initialState", initialState_);

  /*
   * SLQ-MPC settings
   */
  slqSettings_.loadSettings(taskFile);
  mpcSettings_.loadSettings(taskFile);

  /*
   * Cartpole parameters
   */
  CartPoleParameters cartPoleParameters;
  cartPoleParameters.loadSettings(taskFile);

  /*
   * Dynamics
   */
  cartPoleSystemDynamicsPtr_.reset(new CartPoleSytemDynamics(cartPoleParameters));
  cartPoleSystemDynamicsPtr_->initialize("cartpole_dynamics", libraryFolder_, true, true);

  /*
   * Rollout
   */
  Rollout_Settings rolloutSettings;
  rolloutSettings.loadSettings(taskFile, "slq.rollout");
  ddpCartPoleRolloutPtr_.reset(new TimeTriggeredRollout(*cartPoleSystemDynamicsPtr_, rolloutSettings));

  /*
   * Cost function
   */
  loadData::loadEigenMatrix(taskFile, "Q", qm_);
  loadData::loadEigenMatrix(taskFile, "R", rm_);
  loadData::loadEigenMatrix(taskFile, "Q_final", qmFinal_);
  loadData::loadEigenMatrix(taskFile, "x_final", xFinal_);

  xNominal_ = xFinal_;
  uNominal_ = vector_t::Zero(INPUT_DIM_);

  std::cerr << "Q:  \n" << qm_ << std::endl;
  std::cerr << "R:  \n" << rm_ << std::endl;
  std::cerr << "Q_final:\n" << qmFinal_ << std::endl;
  std::cerr << "x_init:   " << initialState_.transpose() << std::endl;
  std::cerr << "x_final:  " << xFinal_.transpose() << std::endl;

  cartPoleCostPtr_.reset(new QuadraticCostFunction(qm_, rm_, xNominal_, uNominal_, qmFinal_, xFinal_));

  /*
   * Constraints
   */
  cartPoleConstraintPtr_.reset(new ConstraintBase());

  /*
   * Initialization
   */
  cartPoleOperatingPointPtr_.reset(new OperatingPoints(initialState_, vector_t::Zero(INPUT_DIM_)));

  /*
   * Time partitioning which defines the time horizon and the number of data partitioning
   */
  ocs2::loadData::loadPartitioningTimes(taskFile, timeHorizon_, numPartitions_, true);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<MPC_SLQ> CartPoleInterface::getMpc() {
  return std::unique_ptr<MPC_SLQ>(new MPC_SLQ(ddpCartPoleRolloutPtr_.get(), cartPoleSystemDynamicsPtr_.get(), cartPoleConstraintPtr_.get(),
                                              cartPoleCostPtr_.get(), cartPoleOperatingPointPtr_.get(), timeHorizon_, numPartitions_,
                                              slqSettings_, mpcSettings_));
}

}  // namespace cartpole
}  // namespace ocs2
