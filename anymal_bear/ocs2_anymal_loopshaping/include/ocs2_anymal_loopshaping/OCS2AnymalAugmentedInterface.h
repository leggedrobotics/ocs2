//
// Created by ruben on 25.07.18.
//

#ifndef OCS2ANYMALAUGMENTEDINTERFACE_H_
#define OCS2ANYMALAUGMENTEDINTERFACE_H_

#include <ocs2_quadruped_augmented_interface/OCS2QuadrupedAugmentedInterface.h>

// Loopshaping
#include <ocs2_core/loopshaping/Loopshaping.h>

// Anymal
#include <ocs2_switched_model_interface/constraint/ComKinoConstraintBaseAd.h>
#include <ocs2_switched_model_interface/cost/SwitchedModelCostBase.h>
#include <ocs2_switched_model_interface/dynamics/ComKinoSystemDynamicsAd.h>
#include <ocs2_switched_model_interface/initialization/ComKinoOperatingPointsBase.h>

namespace anymal {

template<
    size_t STATE_DIM,
    size_t INPUT_DIM,
    size_t SYSTEM_STATE_DIM,
    size_t SYSTEM_INPUT_DIM,
    size_t FILTER_STATE_DIM,
    size_t FILTER_INPUT_DIM,
    size_t JOINT_COORD_SIZE>
class OCS2AnymalAugmentedInterface : public switched_model::OCS2QuadrupedAugmentedInterface<
    STATE_DIM, INPUT_DIM,
    SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM,
    FILTER_STATE_DIM, FILTER_INPUT_DIM,
    JOINT_COORD_SIZE> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<OCS2AnymalAugmentedInterface> Ptr;

  static constexpr size_t JOINT_COORD_SIZE_ = JOINT_COORD_SIZE;
  static constexpr size_t STATE_DIM_ = STATE_DIM;
  static constexpr size_t INPUT_DIM_ = INPUT_DIM;

  using BASE = switched_model::OCS2QuadrupedAugmentedInterface<
      STATE_DIM, INPUT_DIM,
      SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM,
      FILTER_STATE_DIM, FILTER_INPUT_DIM,
      JOINT_COORD_SIZE>;
  using typename BASE::BASE::logic_rules_t;
  using typename BASE::BASE::logic_rules_ptr_t;
  using typename BASE::BASE::mode_sequence_template_t;
  using typename BASE::BASE::slq_base_ptr_t;
  using typename BASE::BASE::slq_mp_t;
  using typename BASE::BASE::slq_t;
  using typename BASE::BASE::mpc_ptr_t;
  using typename BASE::BASE::mpc_t;
  using typename BASE::BASE::state_vector_t;
  using typename BASE::BASE::input_vector_t;
  using typename BASE::controlled_system_base_ptr_t;
  using typename BASE::controlled_system_base_t;
  using typename BASE::rollout_base_t;
  using typename BASE::time_triggered_rollout_t;
//  using mpc_ocs2_t = ocs2::MPC_OCS2<STATE_DIM, INPUT_DIM>;

  using system_dynamics_t = ocs2::LoopshapingDynamics<
      STATE_DIM, INPUT_DIM,
      SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM,
      FILTER_STATE_DIM, FILTER_INPUT_DIM>;
  using system_dynamics_derivative_t = ocs2::LoopshapingDynamicsDerivative<
      STATE_DIM, INPUT_DIM,
      SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM,
      FILTER_STATE_DIM, FILTER_INPUT_DIM>;
  using constraint_t = ocs2::LoopshapingConstraint<
      STATE_DIM, INPUT_DIM,
      SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM,
      FILTER_STATE_DIM, FILTER_INPUT_DIM>;
  using operating_point_t = ocs2::LoopshapingOperatingPoint<
      STATE_DIM, INPUT_DIM,
      SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM,
      FILTER_STATE_DIM, FILTER_INPUT_DIM>;
  using cost_function_t = ocs2::LoopshapingCost<
      STATE_DIM, INPUT_DIM,
      SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM,
      FILTER_STATE_DIM, FILTER_INPUT_DIM>;
  using filter_dynamics_t = ocs2::LoopshapingFilterDynamics<
      STATE_DIM, INPUT_DIM,
      SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM,
      FILTER_STATE_DIM, FILTER_INPUT_DIM>;

  using anymal_system_dynamics_t = switched_model::ComKinoSystemDynamicsAd;
  using anymal_system_dynamics_derivative_t = switched_model::ComKinoSystemDynamicsAd;
  using anymal_constraint_t = switched_model::ComKinoConstraintBaseAd;
  using anymal_cost_function_t = switched_model::SwitchedModelCostBase;
  using anymal_operating_point_t = switched_model::ComKinoOperatingPointsBase;

  OCS2AnymalAugmentedInterface(const std::string &pathToConfigFolder);

  ~OCS2AnymalAugmentedInterface() = default;

  /**
   * setup all optimizes
   */
  void setupOptimizer(const logic_rules_ptr_t& logicRulesPtr, const mode_sequence_template_t* modeSequenceTemplatePtr,
                      slq_base_ptr_t& slqPtr, mpc_ptr_t& mpcPtr) override;

  void initializeFilterState();

  std::unique_ptr<filter_dynamics_t> getFilterDynamics() { return std::unique_ptr<filter_dynamics_t>(new filter_dynamics_t(loopshapingDefinition_)); };

  std::shared_ptr<ocs2::LoopshapingDefinition> getLoopshapingDefinition() { return loopshapingDefinition_; };

  const system_dynamics_t& getDynamics() const override { return *dynamicsPtr_; }

  const system_dynamics_derivative_t& getDynamicsDerivatives() const override { return *dynamicsDerivativesPtr_; }

  const cost_function_t& getCost() const override { return *costFunctionPtr_; }

  const constraint_t* getConstraintPtr() const override { return constraintsPtr_.get(); }

  const rollout_base_t& getRollout() const override { return *timeTriggeredRolloutPtr_; }

 protected:
  std::unique_ptr<anymal_system_dynamics_t> anymalDynamicsPtr_;
  std::unique_ptr<anymal_system_dynamics_derivative_t> anymalDynamicsDerivativesPtr_;
  std::unique_ptr<anymal_constraint_t> anymalConstraintsPtr_;
  std::unique_ptr<anymal_cost_function_t> anymalCostFunctionPtr_;
  std::unique_ptr<anymal_operating_point_t> anymalOperatingPointPtr_;
  std::unique_ptr<rollout_base_t> timeTriggeredRolloutPtr_;

  std::unique_ptr<system_dynamics_t> dynamicsPtr_;
  std::unique_ptr<system_dynamics_derivative_t> dynamicsDerivativesPtr_;
  std::unique_ptr<constraint_t> constraintsPtr_;
  std::unique_ptr<cost_function_t> costFunctionPtr_;
  std::unique_ptr<operating_point_t> operatingPointsPtr_;
  std::unique_ptr<filter_dynamics_t> filterDynamicsPtr_;

  using BASE::BASE::initialState_;
  using BASE::BASE::initRbdState_;
  using BASE::Q_system_;
  using BASE::R_system_;
  using BASE::Q_system_final_;
  using BASE::x_system_final_;

  using BASE::loopshapingDefinition_;
  using BASE::BASE::logicRulesPtr_;
  using BASE::BASE::slqPtr_;
  using BASE::BASE::mpcPtr_;
  using BASE::BASE::defaultModeSequenceTemplate_;
  using BASE::BASE::partitioningTimes_;
  using BASE::BASE::modelSettings_;
  using BASE::BASE::slqSettings_;
  using BASE::BASE::mpcSettings_;

};
} // end of namespace anymal

//#include "implementation/OCS2AnymalAugmentedInterface.tpp"

#endif /* OCS2ANYMALAUGMENTEDINTERFACE_H_ */
