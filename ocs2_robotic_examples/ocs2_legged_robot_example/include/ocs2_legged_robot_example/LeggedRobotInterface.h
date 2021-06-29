/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

// ocs2
#include <ocs2_core/misc/Display.h>
#include <ocs2_mpc/MPC_DDP.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <ocs2_centroidal_model/PinocchioCentroidalDynamicsAD.h>

// ocs2_legged_robot_example
#include "ocs2_legged_robot_example/constraint/LeggedRobotConstraintAD.h"
#include "ocs2_legged_robot_example/cost/LeggedRobotCost.h"
#include "ocs2_legged_robot_example/dynamics/LeggedRobotDynamicsAD.h"
#include "ocs2_legged_robot_example/initialization/LeggedRobotInitializer.h"
#include "ocs2_legged_robot_example/logic/GaitReceiver.h"
#include "ocs2_legged_robot_example/logic/SwitchedModelModeScheduleManager.h"

/**
 * LeggedRobotInterface class
 * General interface for mpc implementation on the legged robot model
 */
namespace ocs2 {
namespace legged_robot {

class LeggedRobotInterface final : public RobotInterface {
 public:
  LeggedRobotInterface(const std::string& taskFileFolderName, const ::urdf::ModelInterfaceSharedPtr& urdfTree);

  ~LeggedRobotInterface() override = default;

  void loadSettings(const std::string& taskFile);

  ocs2::mpc::Settings& mpcSettings() { return mpcSettings_; }

  ModelSettings& modelSettings() { return modelSettings_; }

  const ModelSettings& modelSettings() const { return modelSettings_; }

  const ocs2::rollout::Settings& rolloutSettings() const { return rolloutSettings_; }

  void setupOptimizer(const std::string& taskFile);

  vector_t getInitialState() { return initialState_; }

  ocs2::MPC_DDP& getMpc() { return *mpcPtr_; }

  const SystemDynamicsBase& getDynamics() const override { return *dynamicsPtr_; }

  const CostFunctionBase& getCost() const override { return *costPtr_; }

  const ConstraintBase* getConstraintPtr() const override { return constraintsPtr_.get(); }

  const LeggedRobotInitializer& getInitializer() const override { return *initializerPtr_; }

  const RolloutBase& getRollout() { return *rolloutPtr_; }

  /** Gets the solver synchronized modules */
  const std::vector<std::shared_ptr<SolverSynchronizedModule>>& getSynchronizedModules() const { return solverModules_; };

  /** Gets the switched-model mode schedule manager */
  std::shared_ptr<SwitchedModelModeScheduleManager> getSwitchedModelModeScheduleManagerPtr() const { return modeScheduleManagerPtr_; }

  /** Gets the mode schedule manager */
  std::shared_ptr<ocs2::ModeScheduleManager> getModeScheduleManagerPtr() const override { return modeScheduleManagerPtr_; }

  PinocchioInterface& getPinocchioInterface() { return pinocchioInterface_; }
  CentroidalModelPinocchioMapping& getPinocchioMapping() { return *pinocchioMappingPtr_; }

 protected:
  PinocchioInterface pinocchioInterface_;
  std::unique_ptr<CentroidalModelPinocchioMapping> pinocchioMappingPtr_;
  std::unique_ptr<CentroidalModelPinocchioMappingCppAd> pinocchioMappingCppAdPtr_;

  std::string taskFile_;

  vector_t initialState_;

  ocs2::ddp::Settings ddpSettings_;
  ocs2::rollout::Settings rolloutSettings_;
  ocs2::mpc::Settings mpcSettings_;
  ModelSettings modelSettings_;

  std::unique_ptr<ocs2::MPC_DDP> mpcPtr_;

  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::unique_ptr<SystemDynamicsBase> dynamicsPtr_;
  std::unique_ptr<CostFunctionBase> costPtr_;
  std::unique_ptr<ConstraintBase> constraintsPtr_;
  std::unique_ptr<LeggedRobotInitializer> initializerPtr_;

  // switched model interface
  std::shared_ptr<SwitchedModelModeScheduleManager> modeScheduleManagerPtr_;
  std::vector<std::shared_ptr<SolverSynchronizedModule>> solverModules_;
};

}  // namespace legged_robot
}  // namespace ocs2
