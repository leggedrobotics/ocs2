/*
 * ComKinoConstraintBase.h
 *
 *  Created on: Nov 12, 2017
 *      Author: farbod
 */

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>*
ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::clone() const {
  return new ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::initializeModel(logic_rules_machine_t& logicRulesMachine,
                                                                                                   const size_t& partitionIndex,
                                                                                                   const char* algorithmName /*=NULL*/) {
  Base::initializeModel(logicRulesMachine, partitionIndex, algorithmName);

  findActiveSubsystemFnc_ = std::move(logicRulesMachine.getHandleToFindActiveEventCounter(partitionIndex));

  logicRulesPtr_ = logicRulesMachine.getLogicRulesPtr();

  numEventTimes_ = logicRulesMachine.getLogicRulesPtr()->getNumEventTimes();

  if (algorithmName != NULL)
    algorithmName_.assign(algorithmName);
  else
    algorithmName_.clear();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setCurrentStateAndControl(const scalar_t& t,
                                                                                                             const state_vector_t& x,
                                                                                                             const input_vector_t& u) {
  Base::setCurrentStateAndControl(t, x, u);

  auto activeSubsystem = findActiveSubsystemFnc_(t);
  logicRulesPtr_->getMotionPhaseLogics(activeSubsystem, stanceLegs_, zDirectionRefsPtr_);

  // Active friction cone constraint for stanceLegs
  inequalityConstraintCollection_.modifyConstraint("LF_FrictionCone")->setActivity(stanceLegs_[0]);
  inequalityConstraintCollection_.modifyConstraint("RF_FrictionCone")->setActivity(stanceLegs_[1]);
  inequalityConstraintCollection_.modifyConstraint("LH_FrictionCone")->setActivity(stanceLegs_[2]);
  inequalityConstraintCollection_.modifyConstraint("RH_FrictionCone")->setActivity(stanceLegs_[3]);

  // Active foot placement for stance legs
  inequalityConstraintCollection_.modifyConstraint("LF_EEPos")->setActivity(stanceLegs_[0]);
  inequalityConstraintCollection_.modifyConstraint("RF_EEPos")->setActivity(stanceLegs_[1]);
  inequalityConstraintCollection_.modifyConstraint("LH_EEPos")->setActivity(stanceLegs_[2]);
  inequalityConstraintCollection_.modifyConstraint("RH_EEPos")->setActivity(stanceLegs_[3]);
  inequalityConstraintCollection_.template modifyConstraint<EndEffectorPositionConstraint_t>("LF_EEPos")->configure(eePosConSettings_[0]);
  inequalityConstraintCollection_.template modifyConstraint<EndEffectorPositionConstraint_t>("RF_EEPos")->configure(eePosConSettings_[1]);
  inequalityConstraintCollection_.template modifyConstraint<EndEffectorPositionConstraint_t>("LH_EEPos")->configure(eePosConSettings_[2]);
  inequalityConstraintCollection_.template modifyConstraint<EndEffectorPositionConstraint_t>("RH_EEPos")->configure(eePosConSettings_[3]);

  // Zero forces active for swing legs
  equalityStateInputConstraintCollection_.modifyConstraint("LF_ZeroForce")->setActivity(!stanceLegs_[0]);
  equalityStateInputConstraintCollection_.modifyConstraint("RF_ZeroForce")->setActivity(!stanceLegs_[1]);
  equalityStateInputConstraintCollection_.modifyConstraint("LH_ZeroForce")->setActivity(!stanceLegs_[2]);
  equalityStateInputConstraintCollection_.modifyConstraint("RH_ZeroForce")->setActivity(!stanceLegs_[3]);

  // Velocity constraint is based on the phase, constraint is always active
  for (int i = 0; i < 4; i++) {
    if (stanceLegs_[i]) {  // in stance: All velocity equal to zero
      eeVelConSettings_[i].desiredWorldEndEffectorVelocity.setZero();
      eeVelConSettings_[i].projection = Eigen::Matrix3d::Identity();
    } else {  // in swing: z-velocity is provided
      eeVelConSettings_[i].desiredWorldEndEffectorVelocity[2] = zDirectionRefsPtr_[i]->calculateVelocity(Base::t_);
      eeVelConSettings_[i].projection.resize(1, 3);
      eeVelConSettings_[i].projection << 0, 0, 1;
    }
  }
  equalityStateInputConstraintCollection_.template modifyConstraint<EndEffectorVelocityConstraint_t>("LF_EEVel")
      ->configure(eeVelConSettings_[0]);
  equalityStateInputConstraintCollection_.template modifyConstraint<EndEffectorVelocityConstraint_t>("RF_EEVel")
      ->configure(eeVelConSettings_[1]);
  equalityStateInputConstraintCollection_.template modifyConstraint<EndEffectorVelocityConstraint_t>("LH_EEVel")
      ->configure(eeVelConSettings_[2]);
  equalityStateInputConstraintCollection_.template modifyConstraint<EndEffectorVelocityConstraint_t>("RH_EEVel")
      ->configure(eeVelConSettings_[3]);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getConstraint1(constraint1_vector_t& g1) {
  size_t numConstraints = numStateInputConstraint(Base::t_);
  g1.segment(0, numConstraints) = equalityStateInputConstraintCollection_.getConstraints().getValueAsVector(Base::t_, Base::x_, Base::u_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
size_t ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::numStateInputConstraint(const scalar_t& time) {
  return equalityStateInputConstraintCollection_.getConstraints().getNumConstraints(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getConstraint2(constraint2_vector_t& g2) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
size_t ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::numStateOnlyConstraint(const scalar_t& time) {
  return 0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getInequalityConstraint(scalar_array_t& h) {
  h = inequalityConstraintCollection_.getConstraints().getValue(Base::t_, Base::x_, Base::u_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
size_t ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::numInequalityConstraint(const scalar_t& time) {
  return inequalityConstraintCollection_.getConstraints().getNumConstraints(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getFinalConstraint2(constraint2_vector_t& g2Final) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
size_t ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::numStateOnlyFinalConstraint(const scalar_t& time) {
  return 0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getConstraint1DerivativesState(
    constraint1_state_matrix_t& C) {
	// TODO(Ruben) : We know this is the first call to any of the derivatives. Solve properly later
  linearStateInputConstraintApproximation_ =
      equalityStateInputConstraintCollection_.getConstraints().getLinearApproximationAsMatrices(Base::t_, Base::x_, Base::u_);
  size_t numConstraints = numStateInputConstraint(Base::t_);
  C.block(0, 0, numConstraints, STATE_DIM) = linearStateInputConstraintApproximation_.derivativeState;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getConstraint1DerivativesControl(
    constraint1_input_matrix_t& D) {
  size_t numConstraints = numStateInputConstraint(Base::t_);
  D.block(0, 0, numConstraints, INPUT_DIM) = linearStateInputConstraintApproximation_.derivativeInput;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getConstraint1DerivativesEventTimes(
    constraint1_vector_array_t& g1DevArray) {
  // set all to zero
  g1DevArray.resize(numEventTimes_);
  for (constraint1_vector_t& g1Dev : g1DevArray) g1Dev.setZero();

  //	size_t nextFreeIndex = 0;
  //	for (size_t j=0; j<NUM_CONTACT_POINTS_; j++) {
  //		// the contact force at swing leg is zero
  //		if (stanceLegs_[j]==false) {
  //			nextFreeIndex += 3;
  //		} else {
  //			// stance foot velocity in the World frame
  //			nextFreeIndex += 3;
  //		}
  //	} // end of j loop
  //
  //	// add the swing legs z direction constraints derivative, if its CPG is provided
  //	for (size_t j=0; j<NUM_CONTACT_POINTS_; j++) {
  //		if (stanceLegs_[j]==false && zDirectionRefsPtr_[j]!=nullptr) {
  //			const int& startTimesIndex = startTimesIndices_[j][activeSubsystem_];
  //			g1DevArray[startTimesIndex](nextFreeIndex) = -options_.zDirectionVelocityWeight_ *
  //					zDirectionRefsPtr_[j]->calculateStartTimeDerivative(Base::t_);
  //			const int& finalTimesIndex = finalTimesIndices_[j][activeSubsystem_];
  //			g1DevArray[finalTimesIndex](nextFreeIndex) = -options_.zDirectionVelocityWeight_ *
  //					zDirectionRefsPtr_[j]->calculateFinalTimeDerivative(Base::t_);
  //			nextFreeIndex++;
  //		}
  //	} // end of j loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getConstraint2DerivativesState(
    constraint2_state_matrix_t& F) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getInequalityConstraintDerivativesState(
    state_vector_array_t& dhdx) {
  // TODO(Ruben) : We know this is the first call to any of the derivatives. Solve properly later
  quadraticInequalityConstraintApproximation_ =
      inequalityConstraintCollection_.getConstraints().getQuadraticApproximation(Base::t_, Base::x_, Base::u_);
  dhdx = quadraticInequalityConstraintApproximation_.derivativeState;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getInequalityConstraintDerivativesInput(
    switched_model::ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::input_vector_array_t& dhdu) {
  dhdu = quadraticInequalityConstraintApproximation_.derivativeInput;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getInequalityConstraintSecondDerivativesState(
    switched_model::ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::state_matrix_array_t& ddhdxdx) {
  ddhdxdx = quadraticInequalityConstraintApproximation_.secondDerivativesState;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getInequalityConstraintSecondDerivativesInput(
    switched_model::ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::input_matrix_array_t& ddhdudu) {
  ddhdudu = quadraticInequalityConstraintApproximation_.secondDerivativesInput;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getInequalityConstraintDerivativesInputState(
    switched_model::ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::input_state_matrix_array_t& ddhdudx) {
  ddhdudx = quadraticInequalityConstraintApproximation_.derivativesInputState;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getFinalConstraint2DerivativesState(
    constraint2_state_matrix_t& F) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setStanceLegs(const contact_flag_t& stanceLegs) {
  stanceLegs_ = stanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getStanceLegs(contact_flag_t& stanceLegs) {
  stanceLegs = stanceLegs_;
}

}  // end of namespace switched_model
