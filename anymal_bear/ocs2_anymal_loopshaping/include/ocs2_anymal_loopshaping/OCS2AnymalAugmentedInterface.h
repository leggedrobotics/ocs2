//
// Created by ruben on 25.07.18.
//

#ifndef OCS2ANYMALAUGMENTEDINTERFACE_H_
#define OCS2ANYMALAUGMENTEDINTERFACE_H_

#include <ocs2_quadruped_augmented_interface/OCS2QuadrupedAugmentedInterface.h>

// Loopshaping
#include <ocs2_core/loopshaping/Loopshaping.h>

// Anymal
#include <ocs2_anymal_switched_model/dynamics/AnymalSystemDynamicsAd.h>
#include <ocs2_anymal_switched_model/constraint/AnymalComKinoConstraintAd.h>
#include <ocs2_anymal_switched_model/cost/AnymalCost.h>
#include <ocs2_anymal_switched_model/initialization/AnymalComKinoOperatingPoints.h>

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
  using typename BASE::BASE::generalized_coordinate_t;
  using typename BASE::BASE::contact_flag_t;
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

  using anymal_system_dynamics_t = ComKinoSystemDynamicsAd;
  using anymal_system_dynamics_derivative_t = ComKinoSystemDynamicsAd;
  using anymal_constraint_t = AnymalComKinoConstraintAd;
  using anymal_cost_funtion_t = AnymalCost;
  using anymal_operating_point_t = AnymalComKinoOperatingPoints;

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
  typename anymal_system_dynamics_t::Ptr anymalDynamicsPtr_;
  typename anymal_system_dynamics_derivative_t::Ptr anymalDynamicsDerivativesPtr_;
  typename anymal_constraint_t::Ptr anymalConstraintsPtr_;
  typename anymal_cost_funtion_t::Ptr anymalCostFunctionPtr_;
  typename anymal_operating_point_t::Ptr anymalOperatingPointPtr_;
  //rollout
  std::unique_ptr<rollout_base_t> timeTriggeredRolloutPtr_;

  typename system_dynamics_t::Ptr dynamicsPtr_;
  typename system_dynamics_derivative_t::Ptr dynamicsDerivativesPtr_;
  typename constraint_t::Ptr constraintsPtr_;
  typename cost_function_t::Ptr costFunctionPtr_;
  typename operating_point_t::Ptr operatingPointsPtr_;
  typename filter_dynamics_t::Ptr filterDynamicsPtr_;

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
