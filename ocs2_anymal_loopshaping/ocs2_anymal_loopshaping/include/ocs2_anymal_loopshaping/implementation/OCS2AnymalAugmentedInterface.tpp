//
// Created by ruben on 25.07.18.
//
#include <ocs2_anymal_loopshaping/OCS2AnymalAugmentedInterface.h>

#include <ocs2_anymal_switched_model/kinematics/AnymalKinematics.h>
#include <ocs2_anymal_switched_model/dynamics/AnymalCom.h>

namespace anymal {
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM, size_t FILTER_STATE_DIM,
          size_t FILTER_INPUT_DIM, size_t JOINT_COORD_SIZE>
OCS2AnymalAugmentedInterface<STATE_DIM, INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM,
                             JOINT_COORD_SIZE>::OCS2AnymalAugmentedInterface(const std::string& pathToConfigFolder)
    : switched_model::OCS2QuadrupedAugmentedInterface<STATE_DIM, INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM,
                                                      FILTER_INPUT_DIM, JOINT_COORD_SIZE>(AnymalKinematics(), AnymalCom(),
                                                                                          pathToConfigFolder) {
  initializeFilterState();

  // Prepare filter dynamics
  filterDynamicsPtr_.reset(new filter_dynamics_t(loopshapingDefinition_));

  // set up optimizers
  setupOptimizer(logicRulesPtr_, &defaultModeSequenceTemplate_, slqPtr_, mpcPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM, size_t FILTER_STATE_DIM,
          size_t FILTER_INPUT_DIM, size_t JOINT_COORD_SIZE>
void OCS2AnymalAugmentedInterface<STATE_DIM, INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM,
                                  JOINT_COORD_SIZE>::initializeFilterState() {
  AnymalCom anymalCom;
  const auto totalWeight = anymalCom.totalMass() * 9.81;
  typename system_dynamics_t::system_input_vector_t uSystemForWeightCompensation;
  uSystemForWeightCompensation.setZero();
  size_t numLegs;
  for (size_t i = 0; i < numLegs; i++) {
    uSystemForWeightCompensation(3 * i + 2) = totalWeight / numLegs;
  }

  typename system_dynamics_t::filter_state_vector_t initialFilterState;
  typename system_dynamics_t::filter_input_vector_t initialFilterInput;
  loopshapingDefinition_->getFilterEquilibrium(uSystemForWeightCompensation, initialFilterState, initialFilterInput);

  // Also filter state here
  typename system_dynamics_t::system_state_vector_t initialSystemState;
  loopshapingDefinition_->getSystemState(initialState_, initialSystemState);
  loopshapingDefinition_->concatenateSystemAndFilterState(initialSystemState, initialFilterState, initialState_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM, size_t FILTER_STATE_DIM,
          size_t FILTER_INPUT_DIM, size_t JOINT_COORD_SIZE>
void OCS2AnymalAugmentedInterface<STATE_DIM, INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM,
                                  JOINT_COORD_SIZE>::setupOptimizer(const logic_rules_ptr_t& logicRulesPtr,
                                                                    const mode_sequence_template_t* modeSequenceTemplatePtr,
                                                                    slq_base_ptr_t& slqPtr, mpc_ptr_t& mpcPtr) {
  anymalDynamicsPtr_.reset(new anymal_system_dynamics_t(modelSettings_.recompileLibraries_));
  anymalDynamicsDerivativesPtr_.reset(anymalDynamicsPtr_->clone());
  anymalConstraintsPtr_.reset(new anymal_constraint_t(logicRulesPtr, modelSettings_));
  anymalCostFunctionPtr_.reset(new anymal_cost_funtion_t(logicRulesPtr, Q_system_, R_system_, Q_system_final_));
  generalized_coordinate_t defaultCoordinate = initRbdState_.template head<18>();
  anymalOperatingPointPtr_.reset(new anymal_operating_point_t(logicRulesPtr, modelSettings_, defaultCoordinate));
  timeTriggeredRolloutPtr_.reset(new time_triggered_rollout_t(*dynamicsPtr_, BASE::rolloutSettings_));

  dynamicsPtr_ = system_dynamics_t::create(*anymalDynamicsPtr_, loopshapingDefinition_);
  dynamicsDerivativesPtr_ = system_dynamics_derivative_t::create(*anymalDynamicsDerivativesPtr_, loopshapingDefinition_);
  constraintsPtr_ = constraint_t::create(*anymalConstraintsPtr_, loopshapingDefinition_);
  costFunctionPtr_ = cost_function_t::create(*anymalCostFunctionPtr_, loopshapingDefinition_);
  operatingPointsPtr_.reset(new operating_point_t(*anymalOperatingPointPtr_, loopshapingDefinition_));

  // SLQ
  if (slqSettings_.ddpSettings_.useMultiThreading_) {
    slqPtr.reset(new slq_mp_t(timeTriggeredRolloutPtr_.get(), dynamicsDerivativesPtr_.get(), constraintsPtr_.get(), costFunctionPtr_.get(),
                                         operatingPointsPtr_.get(), slqSettings_, logicRulesPtr));
  } else {
    slqPtr.reset(new slq_t(timeTriggeredRolloutPtr_.get(), dynamicsDerivativesPtr_.get(), constraintsPtr_.get(), costFunctionPtr_.get(),
                                      operatingPointsPtr_.get(), slqSettings_, logicRulesPtr));
  }

  // MPC
  if (!modelSettings_.gaitOptimization_) {
    mpcPtr.reset(new mpc_t(timeTriggeredRolloutPtr_.get(), dynamicsDerivativesPtr_.get(), constraintsPtr_.get(), costFunctionPtr_.get(),
                                 operatingPointsPtr_.get(), partitioningTimes_, slqSettings_, mpcSettings_, logicRulesPtr,
                                 modeSequenceTemplatePtr));

  } else {
    //		mpcPtr = mpc_ptr_t( new mpc_ocs2_t(timeTriggeredRolloutPtr_.get(), dynamicsDerivativesPtr_.get(), constraintsPtr_.get(),
    //				costFunctionPtr_.get(), operatingPointsPtr_.get(),
    //				partitioningTimes_,
    //				slqSettings_, mpcSettings_, logicRulesPtr, modeSequenceTemplatePtr));
  }
}

}  // end of namespace anymal
