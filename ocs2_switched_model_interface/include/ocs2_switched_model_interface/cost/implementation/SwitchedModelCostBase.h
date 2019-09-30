/*
 * SwitchedModelCostBase.h
 *
 *  Created on: Nov 23, 2017
 *      Author: farbod
 */

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::SwitchedModelCostBase(const com_model_t& comModel,
                                                                                     std::shared_ptr<const logic_rules_t> logicRulesPtr,
                                                                                     const state_matrix_t& Q, const input_matrix_t& R,
                                                                                     const state_matrix_t& QFinal)
    : BASE(Q, R, state_vector_t::Zero(), input_vector_t::Zero(), QFinal, state_vector_t::Zero()),
      comModelPtr_(comModel.clone()),
      logicRulesPtr_(std::move(logicRulesPtr)) {
  if (!logicRulesPtr_) {
    throw std::runtime_error("[SwitchedModelCostBase] logicRules cannot be a nullptr");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::SwitchedModelCostBase(const SwitchedModelCostBase& rhs)
    : BASE(rhs), comModelPtr_(rhs.comModelPtr_->clone()), logicRulesPtr_(rhs.logicRulesPtr_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>* SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::clone()
    const {
  return new SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x,
                                                                                              const input_vector_t& u) {
  // Get stance configuration
  contact_flag_t stanceLegs;
  size_t index = logicRulesPtr_->getEventTimeCount(t);
  logicRulesPtr_->getContactFlags(index, stanceLegs);

  dynamic_vector_t xNominal;
  dynamic_vector_t uNominal;
  BASE::xNominalFunc_.interpolate(t, xNominal);
  inputFromContactFlags(stanceLegs, uNominal);

  BASE::setCurrentStateAndControl(t, x, u, xNominal, uNominal, xNominal);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::inputFromContactFlags(contact_flag_t contactFlags,
                                                                                          dynamic_vector_t& inputs) {
  // Distribute total mass equally over active stance legs.
  inputs.setZero(INPUT_DIM);

  const scalar_t totalMass = comModelPtr_->totalMass() * 9.81;
  const size_t numEE(4);
  size_t numStanceLegs(0);

  for (size_t i = 0; i < numEE; i++) {
    if (contactFlags[i]) {
      ++numStanceLegs;
    }
  }

  if (numStanceLegs > 0) {
    for (size_t i = 0; i < numEE; i++) {
      if (contactFlags[i]) {
        inputs(3 * i + 2) = totalMass / numStanceLegs;
      }
    }
  }
}

}  // namespace switched_model
