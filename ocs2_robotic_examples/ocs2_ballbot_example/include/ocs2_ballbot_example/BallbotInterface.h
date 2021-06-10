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

// C++
#include <string>

// OCS2
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_mpc/MPC_DDP.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>

// Ballbot
#include "ocs2_ballbot_example/dynamics/BallbotSystemDynamics.h"

// SQP
#include "ocs2_sqp/MultipleShootingSettings.h"

namespace ocs2 {
namespace ballbot {

/**
 * BallbotInterface class
 * General interface for mpc implementation on the ballbot model
 */
class BallbotInterface final : public RobotInterface {
 public:
  /**
   * Constructor
   * @param [in] taskFileFolderName: The name of the folder containing task file
   */
  explicit BallbotInterface(const std::string& taskFileFolderName);

  /**
   * Destructor
   */
  ~BallbotInterface() override = default;

  const vector_t& getInitialState() { return initialState_; }

  multiple_shooting::Settings& sqpSettings() { return sqpSettings_; }

  ddp::Settings& ddpSettings() { return ddpSettings_; }

  mpc::Settings& mpcSettings() { return mpcSettings_; }

  std::unique_ptr<MPC_DDP> getMpc();

  const OptimalControlProblem& getOptimalControlProblem() const override { return *problemPtr_; }

  const RolloutBase& getRollout() const { return *rolloutPtr_; }

  const Initializer& getInitializer() const override { return *ballbotInitializerPtr_; }

 protected:
  /**
   * Load the settings from the path file.
   *
   * @param [in] taskFile: Task's file full path.
   */
  void loadSettings(const std::string& taskFile);

  /**************
   * Variables
   **************/
  std::string taskFile_;
  std::string libraryFolder_;

  ddp::Settings ddpSettings_;
  mpc::Settings mpcSettings_;
  multiple_shooting::Settings sqpSettings_;

  std::unique_ptr<OptimalControlProblem> problemPtr_;
  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::unique_ptr<Initializer> ballbotInitializerPtr_;

  vector_t initialState_{STATE_DIM};
};

}  // namespace ballbot
}  // namespace ocs2
