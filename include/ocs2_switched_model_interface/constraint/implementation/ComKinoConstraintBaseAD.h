/*
 * ComKinoConstraintBaseAD.h
 *
 *  Created on: Nov 12, 2017
 *      Author: farbod
 */

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>*
ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::clone() const {
  return new ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::setCurrentStateAndControl(const scalar_t& t,
                                                                                                             const state_vector_t& x,
                                                                                                             const input_vector_t& u) {
  Base::setCurrentStateAndControl(t, x, u);
  numEventTimes_ = logicRulesPtr_->getNumEventTimes();
  auto activeSubsystem = logicRulesPtr_->getEventTimeCount(t);
  logicRulesPtr_->getMotionPhaseLogics(activeSubsystem, stanceLegs_, zDirectionRefsPtr_);

  for (int i = 0; i < NUM_CONTACT_POINTS_; i++) {
    auto footName = feetNames[i];

    // Active friction cone constraint for stanceLegs
    inequalityConstraintCollection_.modifyConstraint(footName + "_FrictionCone")->setActivity(stanceLegs_[i]);

    // Zero forces active for swing legs
    equalityStateInputConstraintCollection_.modifyConstraint(footName + "_ZeroForce")->setActivity(!stanceLegs_[i]);

    // Active foot placement for stance legs
    auto EEPosConstraint = inequalityConstraintCollection_.template modifyConstraint<EndEffectorPositionConstraint_t>(footName + "_EEPos");
    auto EEVelConstraint = equalityStateInputConstraintCollection_.template modifyConstraint<EndEffectorVelocityConstraint_t>(footName + "_EEVel");

    double constraintScale = options_.zDirectionPositionWeight_;
    if (options_.zDirectionEqualityConstraint_) {
      // No position constraints
      EEPosConstraint->setActivity(false);

      // Velocity constraint
      EEVelConstraint->setActivity(true);
      if (stanceLegs_[i]) {  // in stance: All velocity equal to zero
        eeVelConSettings_[i].b = Eigen::Vector3d::Zero();
        eeVelConSettings_[i].A = Eigen::Matrix3d::Identity();
      } else {  // in swing: z-velocity is provided
        eeVelConSettings_[i].b.resize(1);
        eeVelConSettings_[i].A.resize(1, 3);
        eeVelConSettings_[i].b << -zDirectionRefsPtr_[i]->calculateVelocity(Base::t_);
        eeVelConSettings_[i].A << 0, 0, 1;
      }
      EEVelConstraint->configure(eeVelConSettings_[i]);
    } else {
      // Position constraints in both stance and swing
      EEPosConstraint->setActivity(true);
      if (stanceLegs_[i]) {
        Eigen::MatrixXd planarPolytopes = constraintScale * switched_model::toHalfSpaces(polytopes[i]);
        eePosConSettings_[i].Ab.resize(planarPolytopes.rows() + 2, planarPolytopes.cols());
        eePosConSettings_[i].Ab << planarPolytopes,
            0.0, 0.0, constraintScale, 0.0,
            0.0, 0.0, -constraintScale, 0.0;
      } else {
        // Swing height control
        eePosConSettings_[i].Ab.resize(1, 4);
        eePosConSettings_[i].Ab <<
                                0.0, 0.0, constraintScale, -constraintScale * zDirectionRefsPtr_[i]->calculatePosition(Base::t_);
      }
      EEPosConstraint->configure(eePosConSettings_[i]);

      // Velocity only in stance
      EEVelConstraint->setActivity(stanceLegs_[i]);
      if (stanceLegs_[i]) {  // in stance: All velocity equal to zero
        eeVelConSettings_[i].b = Eigen::Vector3d::Zero();
        eeVelConSettings_[i].A = Eigen::Matrix3d::Identity();
        EEVelConstraint->configure(eeVelConSettings_[i]);
      }
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getConstraint1(constraint1_vector_t& g1) {
  size_t numConstraints = numStateInputConstraint(Base::t_);
  g1.segment(0, numConstraints) = equalityStateInputConstraintCollection_.getConstraints().getValueAsVector(Base::t_, Base::x_, Base::u_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
size_t ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::numStateInputConstraint(const scalar_t& time) {
  return equalityStateInputConstraintCollection_.getConstraints().getNumConstraints(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getConstraint2(constraint2_vector_t& g2) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
size_t ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::numStateOnlyConstraint(const scalar_t& time) {
  return 0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getInequalityConstraint(scalar_array_t& h) {
  h = inequalityConstraintCollection_.getConstraints().getValue(Base::t_, Base::x_, Base::u_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
size_t ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::numInequalityConstraint(const scalar_t& time) {
  return inequalityConstraintCollection_.getConstraints().getNumConstraints(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getFinalConstraint2(constraint2_vector_t& g2Final) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
size_t ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::numStateOnlyFinalConstraint(const scalar_t& time) {
  return 0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getConstraint1DerivativesState(
    constraint1_state_matrix_t& C) {
	// TODO(Ruben) : We know this is the first call to any of the derivatives. Solve properly later
  linearStateInputConstraintApproximation_ =
      equalityStateInputConstraintCollection_.getConstraints().getLinearApproximationAsMatrices(Base::t_, Base::x_, Base::u_);
  size_t numConstraints = numStateInputConstraint(Base::t_);
  C.block(0, 0, numConstraints, STATE_DIM) = linearStateInputConstraintApproximation_.derivativeState.block(0, 0, numConstraints, STATE_DIM);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getConstraint1DerivativesControl(
    constraint1_input_matrix_t& D) {
  size_t numConstraints = numStateInputConstraint(Base::t_);
  D.block(0, 0, numConstraints, INPUT_DIM) = linearStateInputConstraintApproximation_.derivativeInput.block(0, 0, numConstraints, INPUT_DIM);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getConstraint1DerivativesEventTimes(
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
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getConstraint2DerivativesState(
    constraint2_state_matrix_t& F) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getInequalityConstraintDerivativesState(
    state_vector_array_t& dhdx) {
  // TODO(Ruben) : We know this is the first call to any of the derivatives. Solve properly later
  quadraticInequalityConstraintApproximation_ =
      inequalityConstraintCollection_.getConstraints().getQuadraticApproximation(Base::t_, Base::x_, Base::u_);
  dhdx = std::move(quadraticInequalityConstraintApproximation_.derivativeState);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getInequalityConstraintDerivativesInput(
    switched_model::ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::input_vector_array_t& dhdu) {
  dhdu = std::move(quadraticInequalityConstraintApproximation_.derivativeInput);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getInequalityConstraintSecondDerivativesState(
    switched_model::ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::state_matrix_array_t& ddhdxdx) {
  ddhdxdx = std::move(quadraticInequalityConstraintApproximation_.secondDerivativesState);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getInequalityConstraintSecondDerivativesInput(
    switched_model::ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::input_matrix_array_t& ddhdudu) {
  ddhdudu = std::move(quadraticInequalityConstraintApproximation_.secondDerivativesInput);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getInequalityConstraintDerivativesInputState(
    switched_model::ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::input_state_matrix_array_t& ddhdudx) {
  ddhdudx = std::move(quadraticInequalityConstraintApproximation_.derivativesInputState);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getFinalConstraint2DerivativesState(
    constraint2_state_matrix_t& F) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::setStanceLegs(const contact_flag_t& stanceLegs) {
  stanceLegs_ = stanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getStanceLegs(contact_flag_t& stanceLegs) {
  stanceLegs = stanceLegs_;
}

}  // end of namespace switched_model
