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
#include <ros/package.h>

namespace ocs2 {
namespace double_integrator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
DoubleIntegratorInterface::DoubleIntegratorInterface(const std::string& taskFileFolderName) {
  taskFile_ = ros::package::getPath("ocs2_double_integrator_example") + "/config/" + taskFileFolderName + "/task.info";
  std::cerr << "Loading task file: " << taskFile_ << std::endl;

  libraryFolder_ = ros::package::getPath("ocs2_double_integrator_example") + "/auto_generated";
  std::cerr << "Generated library path: " << libraryFolder_ << std::endl;

  // load setting from loading file
  loadSettings(taskFile_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void DoubleIntegratorInterface::loadSettings(const std::string& taskFile) {
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
   * Dynamics
   */
  dim_t::state_matrix_t A;
  A << 0.0, 1.0, 0.0, 0.0;
  dim_t::state_input_matrix_t B;
  B << 0.0, 1.0;
  linearSystemDynamicsPtr_.reset(new DoubleIntegratorDynamics(A, B));
  linearSystemDynamicsDerivativesPtr_.reset(new DoubleIntegratorDynamicsDerivatives(A, B));

  /*
   * Rollout
   */
  Rollout_Settings rolloutSettings;
  rolloutSettings.loadSettings(taskFile, "slq.rollout");
  ddpLinearSystemRolloutPtr_.reset(new time_triggered_rollout_t(*linearSystemDynamicsPtr_, rolloutSettings));

  /*
   * Cost function
   */
  loadData::loadEigenMatrix(taskFile, "Q", Q_);
  loadData::loadEigenMatrix(taskFile, "R", R_);
  loadData::loadEigenMatrix(taskFile, "Q_final", QFinal_);
  loadData::loadEigenMatrix(taskFile, "x_final", xFinal_);
  xNominal_ = dim_t::state_vector_t::Zero();
  uNominal_ = dim_t::input_vector_t::Zero();

  std::cerr << "Q:  \n" << Q_ << std::endl;
  std::cerr << "R:  \n" << R_ << std::endl;
  std::cerr << "Q_final:\n" << QFinal_ << std::endl;
  std::cerr << "x_init:   " << initialState_.transpose() << std::endl;
  std::cerr << "x_final:  " << xFinal_.transpose() << std::endl;
  linearSystemCostPtr_.reset(new DoubleIntegratorCost(Q_, R_, QFinal_));

  /*
   * Constraints
   */
  linearSystemConstraintPtr_.reset(new DoubleIntegratorConstraint);

  /*
   * Initialization
   */
  //	cartPoleOperatingPointPtr_.reset(new CartPoleOperatingPoint(dim_t::state_vector_t::Zero(), dim_t::input_vector_t::Zero()));
  linearSystemOperatingPointPtr_.reset(new DoubleIntegratorOperatingPoint(initialState_, dim_t::input_vector_t::Zero()));

  /*
   * Time partitioning which defines the time horizon and the number of data partitioning
   */
  dim_t::scalar_t timeHorizon;
  ocs2::loadData::loadPartitioningTimes(taskFile, timeHorizon, numPartitions_, partitioningTimes_, true);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<DoubleIntegratorInterface::mpc_t> DoubleIntegratorInterface::getMpc() {
  return std::unique_ptr<mpc_t>(new mpc_t(ddpLinearSystemRolloutPtr_.get(), linearSystemDynamicsDerivativesPtr_.get(),
                                          linearSystemConstraintPtr_.get(), linearSystemCostPtr_.get(),
                                          linearSystemOperatingPointPtr_.get(), partitioningTimes_, slqSettings_, mpcSettings_));
}

}  // namespace double_integrator
}  // namespace ocs2
