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

#pragma once

// C++
#include <cstdlib>
#include <iostream>
#include <string>

// OCS2
#include <ocs2_core/Dimensions.h>
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/initialization/SystemOperatingPoint.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include <ocs2_mpc/MPC_SLQ.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>

// Double Integrator
#include "ocs2_double_integrator_example/cost/DoubleIntegratorCost.h"
#include "ocs2_double_integrator_example/definitions.h"
#include "ocs2_double_integrator_example/dynamics/DoubleIntegratorDynamics.h"
#include "ocs2_double_integrator_example/dynamics/DoubleIntegratorDynamicsDerivatives.h"

namespace ocs2 {
namespace double_integrator {

class DoubleIntegratorInterface final : public RobotInterface<double_integrator::STATE_DIM_, double_integrator::INPUT_DIM_> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using dim_t = ocs2::Dimensions<double_integrator::STATE_DIM_, double_integrator::INPUT_DIM_>;

  using DoubleIntegratorConstraint = ocs2::ConstraintBase<dim_t::STATE_DIM_, dim_t::INPUT_DIM_>;
  using DoubleIntegratorOperatingPoint = ocs2::SystemOperatingPoint<dim_t::STATE_DIM_, dim_t::INPUT_DIM_>;

  using rollout_base_t = RolloutBase<double_integrator::STATE_DIM_, double_integrator::INPUT_DIM_>;
  using time_triggered_rollout_t = TimeTriggeredRollout<double_integrator::STATE_DIM_, double_integrator::INPUT_DIM_>;

  using mpc_t = ocs2::MPC_SLQ<dim_t::STATE_DIM_, dim_t::INPUT_DIM_>;

  /**
   * Constructor
   * @param [in] taskFileFolderName: The name of the folder containing task file
   */
  explicit DoubleIntegratorInterface(const std::string& taskFileFolderName);

  /**
   * Destructor
   */
  ~DoubleIntegratorInterface() override = default;

  const dim_t::state_vector_t& getInitialState() { return initialState_; }
  SLQ_Settings& slqSettings() { return slqSettings_; };
  MPC_Settings& mpcSettings() { return mpcSettings_; };

  std::unique_ptr<mpc_t> getMpc();

  const dim_t::state_vector_t& getXFinal() { return xFinal_; }

  const DoubleIntegratorDynamics& getDynamics() const override { return *linearSystemDynamicsPtr_; }

  const DoubleIntegratorDynamicsDerivatives& getDynamicsDerivatives() const override { return *linearSystemDynamicsDerivativesPtr_; }

  const DoubleIntegratorCost& getCost() const override { return *linearSystemCostPtr_; }

  const rollout_base_t& getRollout() const { return *ddpLinearSystemRolloutPtr_; }

  const DoubleIntegratorOperatingPoint& getOperatingPoints() const override { return *linearSystemOperatingPointPtr_; }

 protected:
  /**
   * Loads the settings from the path file.
   *
   * @param [in] taskFile: Task's file full path.
   */
  void loadSettings(const std::string& taskFile);

  /**************
   * Variables
   **************/
  std::string taskFile_;
  std::string libraryFolder_;

  SLQ_Settings slqSettings_;
  MPC_Settings mpcSettings_;

  std::unique_ptr<rollout_base_t> ddpLinearSystemRolloutPtr_;

  DoubleIntegratorDynamics::Ptr linearSystemDynamicsPtr_;
  DoubleIntegratorDynamicsDerivatives::Ptr linearSystemDynamicsDerivativesPtr_;
  DoubleIntegratorCost::Ptr linearSystemCostPtr_;
  DoubleIntegratorConstraint::Ptr linearSystemConstraintPtr_;
  DoubleIntegratorOperatingPoint::Ptr linearSystemOperatingPointPtr_;

  // cost parameters
  dim_t::state_matrix_t Q_;
  dim_t::input_matrix_t R_;
  dim_t::state_matrix_t QFinal_;
  dim_t::state_vector_t xFinal_;
  dim_t::state_vector_t xNominal_;
  dim_t::input_vector_t uNominal_;

  size_t numPartitions_ = 0;
  dim_t::scalar_array_t partitioningTimes_;
  dim_t::state_vector_t initialState_;
};

}  // namespace double_integrator
}  // namespace ocs2
