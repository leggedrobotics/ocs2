#pragma once

// ocs2_legged_robot_example
#include <ocs2_legged_robot_example/cost/LeggedRobotCost.h>
#include <ocs2_legged_robot_example/initialization/LeggedRobotInitializer.h>
#include <ocs2_legged_robot_example/logic/GaitReceiver.h>
#include <ocs2_legged_robot_example/logic/SwitchedModelModeScheduleManager.h>
//#include <ocs2_legged_robot_example/constraint/LeggedRobotConstraint.h>
#include <ocs2_legged_robot_example/constraint/LeggedRobotConstraintAD.h>
#include <ocs2_legged_robot_example/dynamics/LeggedRobotDynamicsAD.h>

// ocs2_centroidal_model
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <ocs2_centroidal_model/PinocchioCentroidalDynamicsAD.h>

// ocs2
#include <ocs2_core/misc/Display.h>
#include <ocs2_mpc/MPC_DDP.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>

// pinocchio
#include <pinocchio/parsers/urdf/types.hpp>

// ROS
#include <ros/package.h>

/**
 * LeggedRobotInterface class
 * General interface for mpc implementation on the legged robot model
 */
namespace ocs2 {
namespace legged_robot {

class LeggedRobotInterface final : public ocs2::RobotInterface {
 public:
  using Ptr = std::shared_ptr<LeggedRobotInterface>;

  LeggedRobotInterface(const std::string& taskFileFolderName, const ::urdf::ModelInterfaceSharedPtr& urdfTree);
  ~LeggedRobotInterface() = default;

  void loadSettings(const std::string& taskFile);

  ocs2::mpc::Settings& mpcSettings() { return mpcSettings_; }

  ModelSettings& modelSettings() { return modelSettings_; }

  const ModelSettings& modelSettings() const { return modelSettings_; }

  const ocs2::rollout::Settings& rolloutSettings() const { return rolloutSettings_; }

  void setupOptimizer(const std::string& taskFile);

  state_vector_t getInitialState() { return initialState_; }

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

  PinocchioInterface& getPinocchioInterface() { return *pinocchioInterfacePtr_; }

 protected:
  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
  std::unique_ptr<CentroidalModelPinocchioMapping<scalar_t>> pinocchioMappingPtr_;
  std::unique_ptr<CentroidalModelPinocchioMapping<ad_scalar_t>> pinocchioMappingAdPtr_;

  std::string taskFile_;

  state_vector_t initialState_;

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
