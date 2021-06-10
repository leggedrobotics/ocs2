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

#pragma once

// OCS2
#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/cost/QuadraticCostFunction.h>
#include <ocs2_core/initialization/OperatingPoints.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include <ocs2_mpc/MPC_DDP.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>

// Quadrotor
#include "ocs2_quadrotor_example/QuadrotorParameters.h"
#include "ocs2_quadrotor_example/definitions.h"
#include "ocs2_quadrotor_example/dynamics/QuadrotorSystemDynamics.h"

namespace ocs2 {
namespace quadrotor {

class QuadrotorInterface final : public RobotInterface {
 public:
  /**
   * Constructor
   *
   * @param [in] taskFileFolderName: The name of the folder containing task file
   */
  explicit QuadrotorInterface(const std::string& taskFileFolderName);

  /** Destructor */
  ~QuadrotorInterface() override = default;

  const vector_t& getInitialState() { return initialState_; }

  ddp::Settings& ddpSettings() { return ddpSettings_; }

  mpc::Settings& mpcSettings() { return mpcSettings_; }

  std::unique_ptr<MPC_DDP> getMpc();

  const QuadrotorSystemDynamics& getDynamics() const override { return *quadrotorSystemDynamicsPtr_; }

  const QuadraticCostFunction& getCost() const override { return *quadrotorCostPtr_; }

  const RolloutBase& getRollout() const { return *ddpQuadrotorRolloutPtr_; }

  const Initializer& getInitializer() const override { return *quadrotorOperatingPointPtr_; }

 protected:
  /**
   * Load the settings from the path file.
   *
   * @param [in] taskFile: Task's file full path.
   */
  void loadSettings(const std::string& taskFile);

  std::string taskFile_;
  std::string libraryFolder_;

  ddp::Settings ddpSettings_;
  mpc::Settings mpcSettings_;

  std::unique_ptr<RolloutBase> ddpQuadrotorRolloutPtr_;

  std::unique_ptr<QuadrotorSystemDynamics> quadrotorSystemDynamicsPtr_;
  std::unique_ptr<QuadraticCostFunction> quadrotorCostPtr_;
  std::unique_ptr<ConstraintBase> quadrotorConstraintPtr_;
  std::unique_ptr<OperatingPoints> quadrotorOperatingPointPtr_;

  // cost parameters
  matrix_t Q_{STATE_DIM, STATE_DIM};
  matrix_t R_{INPUT_DIM, INPUT_DIM};
  matrix_t QFinal_{STATE_DIM, STATE_DIM};
  vector_t initialState_{STATE_DIM};
};

}  // namespace quadrotor
}  // namespace ocs2
