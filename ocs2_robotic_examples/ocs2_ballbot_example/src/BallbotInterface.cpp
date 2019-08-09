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

#include "ocs2_ballbot_example/BallbotInterface.h"
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

  // MPC
  setupOptimizer(taskFile_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void BallbotInterface::loadSettings(const std::string& taskFile) {
  /*
   * Default initial condition
   */
  loadInitialState(taskFile, initialState_);

  /*
   * SLQ-MPC settings
   */
  slqSettings_.loadSettings(taskFile);
  mpcSettings_.loadSettings(taskFile);
  piSettings_.loadSettings(taskFile);

  /*
   * Dynamics
   */
  // load the flag to generate library files from taskFile
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);
  libraryFilesAreGenerated_ = pt.get<bool>("ballbot_interface.libraryFilesAreGenerated");

  ballbotSystemDynamicsPtr_.reset(new BallbotSystemDynamics(libraryFilesAreGenerated_));

  if (libraryFilesAreGenerated_) {
    ballbotSystemDynamicsPtr_->loadModels("ballbot_dynamics", libraryFolder_);
  } else {
    ballbotSystemDynamicsPtr_->createModels("ballbot_dynamics", libraryFolder_);
  }

  /*
   * Cost function
   */
  ocs2::loadEigenMatrix(taskFile, "Q", Q_);
  ocs2::loadEigenMatrix(taskFile, "R", R_);
  ocs2::loadEigenMatrix(taskFile, "Q_final", QFinal_);
  ocs2::loadEigenMatrix(taskFile, "x_final", xFinal_);
  //	xNominal_ = dim_t::state_vector_t::Zero();
  xNominal_ = xFinal_;
  uNominal_ = dim_t::input_vector_t::Zero();

  std::cerr << "Q:  \n" << Q_ << std::endl;
  std::cerr << "R:  \n" << R_ << std::endl;
  std::cerr << "Q_final:\n" << QFinal_ << std::endl;
  std::cerr << "x_init:   " << initialState_.transpose() << std::endl;
  std::cerr << "x_final:  " << xFinal_.transpose() << std::endl;

  ballbotCostPtr_.reset(new BallbotCost(Q_, R_, xNominal_, uNominal_, QFinal_, xFinal_));

  /*
   * Constraints
   */
  ballbotConstraintPtr_.reset(new ballbotConstraint_t);

  /*
   * Initialization
   */
  //	cartPoleOperatingPointPtr_.reset(new CartPoleOperatingPoint(dim_t::state_vector_t::Zero(), dim_t::input_vector_t::Zero()));
  ballbotOperatingPointPtr_.reset(new ballbotOperatingPoint_t(initialState_, dim_t::input_vector_t::Zero()));

  /*
   * Time partitioning which defines the time horizon and the number of data partitioning
   */
  scalar_t timeHorizon;
  definePartitioningTimes(taskFile, timeHorizon, numPartitions_, partitioningTimes_, true);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void BallbotInterface::setupOptimizer(const std::string& taskFile) {
  mpcPtr_.reset(new mpc_t(ballbotSystemDynamicsPtr_.get(), ballbotSystemDynamicsPtr_.get(), ballbotConstraintPtr_.get(),
                          ballbotCostPtr_.get(), ballbotOperatingPointPtr_.get(), partitioningTimes_, slqSettings_, mpcSettings_));

  std::unique_ptr<BallbotCost> cost(ballbotCostPtr_->clone());
  mpcPi_.reset(
      new mpc_pi_t(ballbotSystemDynamicsPtr_, std::move(cost), *ballbotConstraintPtr_, partitioningTimes_, mpcSettings_, piSettings_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SLQ_Settings& BallbotInterface::slqSettings() { return slqSettings_; }

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
BallbotInterface::mpc_t::Ptr& BallbotInterface::getMPCPtr() { return mpcPtr_; }

}  // namespace ballbot
}  // namespace ocs2
