#include <ocs2_anymal_wheels_switched_model/constraint/AnymalWheelsComKinoConstraintAd.h>

namespace switched_model {

void AnymalWheelsComKinoConstraintAd::initializeConstraintTerms() {
  for (int footIdx = 0; footIdx < NUM_CONTACT_POINTS; footIdx++) {
    auto footName = feetNames[footIdx];

    // Friction cone constraint
    auto frictionCone = std::unique_ptr<ConstraintTerm_t>(new FrictionConeConstraint_t(options_.frictionCoefficient_, 25.0, footIdx));

    // EE force
    auto zeroForceConstraint = std::unique_ptr<ConstraintTerm_t>(new ZeroForceConstraint_t(footIdx));

    // Velocity Constraint
    auto _o_endEffectorVelocityConstraint = std::unique_ptr<ConstraintTerm_t>(new EndEffectorVelocityConstraint_t(
        footIdx, EndEffectorVelocityConstraintSettings_t(), *adComModelPtr_, *adKinematicModelPtr_, options_.recompileLibraries_));
    // EE InFootFrame Velocity Constraint
    auto _f_endEffectorVelocityInFootFrameConstraint = std::unique_ptr<ConstraintTerm_t>(
        new EndEffectorVelocityInFootFrameConstraint_t(footIdx, EndEffectorVelocityInFootFrameConstraintSettings_t(), *adComModelPtr_,
                                                       *adKinematicModelPtr_, options_.recompileLibraries_));

    // Inequalities
    inequalityConstraintCollection_.add(footName + "_FrictionCone", std::move(frictionCone));

    // State input equalities
    equalityStateInputConstraintCollection_.add(footName + "_ZeroForce", std::move(zeroForceConstraint));
    equalityStateInputConstraintCollection_.add(footName + "_o_EEVel", std::move(_o_endEffectorVelocityConstraint));
    equalityStateInputConstraintCollection_.add(footName + "_f_EEVel", std::move(_f_endEffectorVelocityInFootFrameConstraint));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AnymalWheelsComKinoConstraintAd::setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) {
  stateInputConstraintsComputed_ = false;
  inequalityConstraintsComputed_ = false;

  Base::setCurrentStateAndControl(t, x, u);
  numEventTimes_ = logicRulesPtr_->getNumEventTimes();
  auto activeSubsystem = logicRulesPtr_->getEventTimeCount(t);
  logicRulesPtr_->getMotionPhaseLogics(activeSubsystem, stanceLegs_, zDirectionRefsPtr_);

  for (int footIdx = 0; footIdx < NUM_CONTACT_POINTS; footIdx++) {
    auto footName = feetNames[footIdx];

    // Active friction cone constraint for stanceLegs
    inequalityConstraintCollection_.get(footName + "_FrictionCone").setActivity(stanceLegs_[footIdx]);

    // Zero forces active for swing legs
    equalityStateInputConstraintCollection_.get(footName + "_ZeroForce").setActivity(!stanceLegs_[footIdx]);

    // Active foot placement for stance legs
    auto& _o_EEVelConstraint = equalityStateInputConstraintCollection_.template get<EndEffectorVelocityConstraint_t>(footName + "_o_EEVel");
    EndEffectorVelocityConstraintSettings_t _o_eeVelConSettings(1, 3);

    // Rolling InFootFrame Velocity constraint for stance legs
    auto& _f_EEVelInFootFrameConstraint =
        equalityStateInputConstraintCollection_.template get<EndEffectorVelocityInFootFrameConstraint_t>(footName + "_f_EEVel");

    if (stanceLegs_[footIdx]) {
      // EE velocities in lateral direction (y) in foot frame should be zero.
      EndEffectorVelocityInFootFrameConstraintSettings_t _f_eeVelInFootFrameConSettings(1, 3);
      _f_eeVelInFootFrameConSettings.b << 0;
      _f_eeVelInFootFrameConSettings.A << 0, 1, 0;
      _f_EEVelInFootFrameConstraint.configure(_f_eeVelInFootFrameConSettings);
      _f_EEVelInFootFrameConstraint.setActivity(true);
      // The upwards velocity (z) in the world frame should be zero too.
      _o_eeVelConSettings.b << 0;
      _o_eeVelConSettings.A << 0, 0, 1;
    } else {  // in swing: z-velocity is provided
      _f_EEVelInFootFrameConstraint.setActivity(false);
      _o_eeVelConSettings.b << -zDirectionRefsPtr_[footIdx]->calculateVelocity(Base::t_);
      _o_eeVelConSettings.A << 0, 0, 1;
    }
    _o_EEVelConstraint.configure(_o_eeVelConSettings);
    _o_EEVelConstraint.setActivity(true);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t AnymalWheelsComKinoConstraintAd::numStateInputConstraint(const scalar_t& time) {
  return equalityStateInputConstraintCollection_.getNumConstraints(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AnymalWheelsComKinoConstraintAd::getConstraint1(constraint1_vector_t& e) {
  size_t numConstraints = numStateInputConstraint(Base::t_);
  e.head(numConstraints) = equalityStateInputConstraintCollection_.getValueAsVector(Base::t_, Base::x_, Base::u_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AnymalWheelsComKinoConstraintAd::getConstraint2(constraint2_vector_t& h) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t AnymalWheelsComKinoConstraintAd::numStateOnlyConstraint(const scalar_t& time) {
  return 0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AnymalWheelsComKinoConstraintAd::getInequalityConstraint(scalar_array_t& h) {
  h = inequalityConstraintCollection_.getValue(Base::t_, Base::x_, Base::u_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t AnymalWheelsComKinoConstraintAd::numInequalityConstraint(const scalar_t& time) {
  return inequalityConstraintCollection_.getNumConstraints(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AnymalWheelsComKinoConstraintAd::getFinalConstraint2(constraint2_vector_t& h_f) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t AnymalWheelsComKinoConstraintAd::numStateOnlyFinalConstraint(const scalar_t& time) {
  return 0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AnymalWheelsComKinoConstraintAd::getConstraint1DerivativesState(constraint1_state_matrix_t& C) {
  if (!stateInputConstraintsComputed_) {
    linearStateInputConstraintApproximation_ =
      equalityStateInputConstraintCollection_.getLinearApproximationAsMatrices(Base::t_, Base::x_, Base::u_);
    stateInputConstraintsComputed_ = true;
  }
  size_t numConstraints = numStateInputConstraint(Base::t_);
  C.topRows(numConstraints) = linearStateInputConstraintApproximation_.derivativeState.topRows(numConstraints);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AnymalWheelsComKinoConstraintAd::getConstraint1DerivativesControl(constraint1_input_matrix_t& D) {
  if (!stateInputConstraintsComputed_) {
    linearStateInputConstraintApproximation_ =
      equalityStateInputConstraintCollection_.getLinearApproximationAsMatrices(Base::t_, Base::x_, Base::u_);
    stateInputConstraintsComputed_ = true;
  }
  size_t numConstraints = numStateInputConstraint(Base::t_);
  D.topRows(numConstraints) = linearStateInputConstraintApproximation_.derivativeInput.topRows(numConstraints);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AnymalWheelsComKinoConstraintAd::getConstraint1DerivativesEventTimes(constraint1_vector_array_t& g1DevArray) {
  // set all to zero
  g1DevArray.resize(numEventTimes_);
  for (constraint1_vector_t& g1Dev : g1DevArray) {
    g1Dev.setZero();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AnymalWheelsComKinoConstraintAd::getConstraint2DerivativesState(constraint2_state_matrix_t& F) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AnymalWheelsComKinoConstraintAd::getInequalityConstraintDerivativesState(state_vector_array_t& dhdx) {
  if (!inequalityConstraintsComputed_) {
    quadraticInequalityConstraintApproximation_ = inequalityConstraintCollection_.getQuadraticApproximation(Base::t_, Base::x_, Base::u_);
    inequalityConstraintsComputed_ = true;
  }
  dhdx = std::move(quadraticInequalityConstraintApproximation_.derivativeState);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AnymalWheelsComKinoConstraintAd::getInequalityConstraintDerivativesInput(switched_model::AnymalWheelsComKinoConstraintAd::input_vector_array_t& dhdu) {
  if (!inequalityConstraintsComputed_) {
    quadraticInequalityConstraintApproximation_ = inequalityConstraintCollection_.getQuadraticApproximation(Base::t_, Base::x_, Base::u_);
    inequalityConstraintsComputed_ = true;
  }
  dhdu = std::move(quadraticInequalityConstraintApproximation_.derivativeInput);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AnymalWheelsComKinoConstraintAd::getInequalityConstraintSecondDerivativesState(
    switched_model::AnymalWheelsComKinoConstraintAd::state_matrix_array_t& ddhdxdx) {
  if (!inequalityConstraintsComputed_) {
    quadraticInequalityConstraintApproximation_ = inequalityConstraintCollection_.getQuadraticApproximation(Base::t_, Base::x_, Base::u_);
    inequalityConstraintsComputed_ = true;
  }
  ddhdxdx = std::move(quadraticInequalityConstraintApproximation_.secondDerivativesState);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AnymalWheelsComKinoConstraintAd::getInequalityConstraintSecondDerivativesInput(
    switched_model::AnymalWheelsComKinoConstraintAd::input_matrix_array_t& ddhdudu) {
  if (!inequalityConstraintsComputed_) {
    quadraticInequalityConstraintApproximation_ = inequalityConstraintCollection_.getQuadraticApproximation(Base::t_, Base::x_, Base::u_);
    inequalityConstraintsComputed_ = true;
  }
  ddhdudu = std::move(quadraticInequalityConstraintApproximation_.secondDerivativesInput);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AnymalWheelsComKinoConstraintAd::getInequalityConstraintDerivativesInputState(
    switched_model::AnymalWheelsComKinoConstraintAd::input_state_matrix_array_t& ddhdudx) {
  if (!inequalityConstraintsComputed_) {
    quadraticInequalityConstraintApproximation_ = inequalityConstraintCollection_.getQuadraticApproximation(Base::t_, Base::x_, Base::u_);
    inequalityConstraintsComputed_ = true;
  }
  ddhdudx = std::move(quadraticInequalityConstraintApproximation_.derivativesInputState);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AnymalWheelsComKinoConstraintAd::getFinalConstraint2DerivativesState(constraint2_state_matrix_t& F) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AnymalWheelsComKinoConstraintAd::setStanceLegs(const contact_flag_t& stanceLegs) {
  stanceLegs_ = stanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AnymalWheelsComKinoConstraintAd::getStanceLegs(contact_flag_t& stanceLegs) {
  stanceLegs = stanceLegs_;
}

}
// end of namespace switched_model
