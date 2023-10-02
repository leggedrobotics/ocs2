/*
 * QuadrupedInterface.h
 *
 *  Created on: Feb 14, 2018
 *      Author: farbod
 */

#pragma once

#include <ocs2_oc/rollout/RolloutBase.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include <ocs2_robotic_tools/common/RobotInterface.h>

#include <ocs2_switched_model_interface/core/ComModelBase.h>
#include <ocs2_switched_model_interface/core/InverseKinematicsModelBase.h>
#include <ocs2_switched_model_interface/core/KinematicsModelBase.h>
#include <ocs2_switched_model_interface/core/ModelSettings.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/cost/MotionTrackingCost.h>

#include <ocs2_switched_model_interface/logic/DynamicsParametersSynchronizedModule.h>
#include <ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h>

namespace switched_model {

class QuadrupedInterface : public ocs2::RobotInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using com_model_t = ComModelBase<scalar_t>;
  using kinematic_model_t = KinematicsModelBase<scalar_t>;

  using ad_com_model_t = ComModelBase<ad_scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ad_scalar_t>;

  using synchronized_module_t = ocs2::SolverSynchronizedModule;
  using synchronized_module_ptr_array_t = std::vector<std::shared_ptr<synchronized_module_t>>;

  struct Settings {
    ocs2::rollout::Settings rolloutSettings_;
    ModelSettings modelSettings_;
    MotionTrackingCost::Weights trackingWeights_;
    SwingTrajectoryPlannerSettings swingTrajectoryPlannerSettings_;
    TerrainPlane terrainPlane_;
    state_vector_t initialState_;
    Gait defaultGait_;
  };

  /**
   * Constructor
   * @param kinematicModel : Kinematics model
   * @param adKinematicModel : Kinematics model templated on the auto-differentiation scalar
   * @param comModel : Center of mass model
   * @param adComModel : Center of mass model templated on the auto-differentiation scalar
   * @param settings : settings
   * @param jointNames : names of the joints in the order of the state vector
   * @param baseName : names of the model root frame
   */
  QuadrupedInterface(const kinematic_model_t& kinematicModel, const ad_kinematic_model_t& adKinematicModel, const com_model_t& comModel,
                     const ad_com_model_t& adComModel, const InverseKinematicsModelBase* inverseKinematics, Settings settings,
                     std::vector<std::string> jointNames, std::string baseName);

  /** Destructor */
  ~QuadrupedInterface() override = default;

  /** Gets the switched-model mode schedule manager */
  std::shared_ptr<SwitchedModelModeScheduleManager> getSwitchedModelModeScheduleManagerPtr() const { return modeScheduleManagerPtr_; }

  /** Gets the mode schedule manager */
  std::shared_ptr<ocs2::ReferenceManagerInterface> getReferenceManagerPtr() const override { return modeScheduleManagerPtr_; }

  /** Gets the dynamics parameter module*/
  std::shared_ptr<DynamicsParametersSynchronizedModule> getDynamicsParametersSynchronizedModulePtr() const {
    return dynamicsParametersSynchronizedModulePtr_;
  }

  const ocs2::OptimalControlProblem& getOptimalControlProblem() const override { return *problemPtr_; }

  /** Gets inverse kinematic model */
  const InverseKinematicsModelBase* getInverseKinematicModelPtr() const { return inverseKinematicModelPtr_.get(); }

  /** Gets kinematic model */
  const kinematic_model_t& getKinematicModel() const { return *kinematicModelPtr_; }
  const ad_kinematic_model_t& getKinematicModelAd() const { return *adKinematicModelPtr_; }

  /** Gets center of mass model */
  const com_model_t& getComModel() const { return *comModelPtr_; }
  const ad_com_model_t& getComModelAd() const { return *adComModelPtr_; }

  /** Gets the loaded initial state */
  state_vector_t& getInitialState() { return settings_.initialState_; }
  const state_vector_t& getInitialState() const { return settings_.initialState_; }

  /** Access to rollout settings */
  const ocs2::rollout::Settings& rolloutSettings() const { return settings_.rolloutSettings_; }

  /** Access to model settings */
  const ModelSettings& modelSettings() const { return settings_.modelSettings_; };

  /** Access to model names */
  const std::vector<std::string>& getJointNames() const { return jointNames_; };
  const std::string& getBaseName() const { return baseName_; };

  /** Access to cost settings */
  const MotionTrackingCost::Weights& costSettings() const { return settings_.trackingWeights_; };

  /** Gets the rollout class */
  virtual const ocs2::RolloutBase& getRollout() const = 0;

  /** Gets the solver synchronized modules */
  const synchronized_module_ptr_array_t& getSynchronizedModules() const { return solverModules_; };

  /** Append solver sync module to solverModules_*/
  void appendToSynchronizedModules(std::shared_ptr<synchronized_module_t> synchronizedModule) {
    solverModules_.push_back(std::move(synchronizedModule));
  };

  /** Cost approximation at the nominal state and input*/
  virtual const ScalarFunctionQuadraticApproximation& nominalCostApproximation() const = 0;

 protected:
  std::unique_ptr<ocs2::OptimalControlProblem> problemPtr_;

  std::unique_ptr<ocs2::PreComputation> createPrecomputation() const;
  std::unique_ptr<ocs2::StateInputCost> createMotionTrackingCost() const;
  std::unique_ptr<ocs2::StateCost> createMotionTrackingTerminalCost(matrix_t Q) const;
  std::unique_ptr<ocs2::StateCost> createFootPlacementCost() const;
  std::unique_ptr<ocs2::StateCost> createCollisionAvoidanceCost() const;
  std::unique_ptr<ocs2::StateInputCost> createJointLimitsSoftConstraint() const;
  std::unique_ptr<ocs2::StateInputCost> createTorqueLimitsSoftConstraint(const joint_coordinate_t& nominalTorques) const;
  std::unique_ptr<ocs2::SystemDynamicsBase> createDynamics() const;
  std::unique_ptr<ocs2::StateInputConstraint> createZeroForceConstraint(size_t leg) const;
  std::unique_ptr<ocs2::StateInputConstraint> createFootNormalConstraint(size_t leg) const;
  std::unique_ptr<ocs2::StateInputConstraint> createEndEffectorVelocityConstraint(size_t leg) const;
  std::unique_ptr<ocs2::StateInputCost> createFrictionConeCost() const;

 private:
  Settings settings_;
  std::vector<std::string> jointNames_;
  std::string baseName_;
  std::unique_ptr<InverseKinematicsModelBase> inverseKinematicModelPtr_;
  std::unique_ptr<kinematic_model_t> kinematicModelPtr_;
  std::unique_ptr<ad_kinematic_model_t> adKinematicModelPtr_;
  std::unique_ptr<com_model_t> comModelPtr_;
  std::unique_ptr<ad_com_model_t> adComModelPtr_;
  synchronized_module_ptr_array_t solverModules_;
  std::shared_ptr<SwitchedModelModeScheduleManager> modeScheduleManagerPtr_;
  std::shared_ptr<DynamicsParametersSynchronizedModule> dynamicsParametersSynchronizedModulePtr_;
};

/** Load the general quadruped settings from file. */
QuadrupedInterface::Settings loadQuadrupedSettings(const std::string& pathToConfigFile);

}  // end of namespace switched_model
