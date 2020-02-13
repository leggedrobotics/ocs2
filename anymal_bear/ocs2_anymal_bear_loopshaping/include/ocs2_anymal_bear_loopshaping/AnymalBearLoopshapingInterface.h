//
// Created by rgrandia on 13.02.20.
//

#pragma once

// Anymal model
#include <ocs2_switched_model_interface/constraint/ComKinoConstraintBaseAd.h>
#include <ocs2_switched_model_interface/cost/SwitchedModelCostBase.h>
#include <ocs2_switched_model_interface/dynamics/ComKinoSystemDynamicsAd.h>
#include <ocs2_switched_model_interface/initialization/ComKinoOperatingPointsBase.h>

// Loopshaping
#include <ocs2_core/loopshaping/Loopshaping.h>

// Interface
#include <ocs2_quadruped_interface/OCS2QuadrupedInterface.h>

namespace anymal {

class AnymalBearLoopshapingInterface final : public switched_model::OCS2QuadrupedInterface<12, 48, 24> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr size_t STATE_DIM = 48;
  static constexpr size_t INPUT_DIM = 24;
  static constexpr size_t SYSTEM_STATE_DIM = 24;
  static constexpr size_t SYSTEM_INPUT_DIM = 24;
  static constexpr size_t FILTER_STATE_DIM = 24;
  static constexpr size_t FILTER_INPUT_DIM = 24;

  using BASE = switched_model::OCS2QuadrupedInterface<12, 48, 24>;

  using system_dynamics_t =
      ocs2::LoopshapingDynamics<STATE_DIM, INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>;
  using system_dynamics_derivative_t =
      ocs2::LoopshapingDynamicsDerivative<STATE_DIM, INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>;
  using constraint_t =
      ocs2::LoopshapingConstraint<STATE_DIM, INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>;
  using operating_point_t =
      ocs2::LoopshapingOperatingPoint<STATE_DIM, INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>;
  using cost_function_t =
      ocs2::LoopshapingCost<STATE_DIM, INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>;
  using filter_dynamics_t =
      ocs2::LoopshapingFilterDynamics<STATE_DIM, INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>;

  using anymal_system_dynamics_t = switched_model::ComKinoSystemDynamicsAd;
  using anymal_system_dynamics_derivative_t = switched_model::ComKinoSystemDynamicsAd;
  using anymal_constraint_t = switched_model::ComKinoConstraintBaseAd;
  using anymal_cost_function_t = switched_model::SwitchedModelCostBase;
  using anymal_operating_point_t = switched_model::ComKinoOperatingPointsBase;

  using system_state_matrix_t = Eigen::Matrix<double, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM>;
  using system_state_vector_t = Eigen::Matrix<double, SYSTEM_STATE_DIM, 1>;
  using system_input_matrix_t = Eigen::Matrix<double, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM>;

  explicit AnymalBearLoopshapingInterface(const std::string& pathToConfigFolder);

  ~AnymalBearLoopshapingInterface() override = default;

  std::shared_ptr<ocs2::LoopshapingDefinition> getLoopshapingDefinition() { return loopshapingDefinition_; };

  std::unique_ptr<slq_t> getSlq() const override;

  std::unique_ptr<mpc_t> getMpc() const override;

  const system_dynamics_t& getDynamics() const override { return *dynamicsPtr_; }

  const system_dynamics_derivative_t& getDynamicsDerivatives() const override { return *dynamicsDerivativesPtr_; }

  const cost_function_t& getCost() const override { return *costFunctionPtr_; }

  const constraint_t* getConstraintPtr() const override { return constraintsPtr_.get(); }

  const rollout_base_t& getRollout() const override { return *timeTriggeredRolloutPtr_; }

 private:
  std::unique_ptr<system_dynamics_t> dynamicsPtr_;
  std::unique_ptr<system_dynamics_derivative_t> dynamicsDerivativesPtr_;
  std::unique_ptr<constraint_t> constraintsPtr_;
  std::unique_ptr<cost_function_t> costFunctionPtr_;
  std::unique_ptr<operating_point_t> operatingPointsPtr_;
  std::unique_ptr<rollout_base_t> timeTriggeredRolloutPtr_;
  std::unique_ptr<filter_dynamics_t> filterDynamicsPtr_;

  std::unique_ptr<anymal_system_dynamics_t> anymalDynamicsPtr_;
  std::unique_ptr<anymal_system_dynamics_derivative_t> anymalDynamicsDerivativesPtr_;
  std::unique_ptr<anymal_constraint_t> anymalConstraintsPtr_;
  std::unique_ptr<anymal_cost_function_t> anymalCostFunctionPtr_;
  std::unique_ptr<anymal_operating_point_t> anymalOperatingPointPtr_;

  std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition_;

  system_state_matrix_t Q_system_, Q_system_final_;
  system_input_matrix_t R_system_;
};

}  // namespace anymal
