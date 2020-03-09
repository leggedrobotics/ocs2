#include <ocs2_anymal_wheels_switched_model/constraint/AnymalWheelsComKinoConstraintAd.h>

namespace switched_model {

AnymalWheelsComKinoConstraintAd::AnymalWheelsComKinoConstraintAd(const ad_kinematic_model_t& adKinematicModel, const ad_com_model_t& adComModel,
    std::shared_ptr<const logic_rules_t> logicRulesPtr, const ModelSettings& options)
  : Base(adKinematicModel, adComModel, logicRulesPtr, options, false, false)
{
  initializeConstraintTerms();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
AnymalWheelsComKinoConstraintAd* AnymalWheelsComKinoConstraintAd::clone() const {
  return new AnymalWheelsComKinoConstraintAd(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
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
    auto _f_endEffectorVelocityInFootFrameConstraint = std::unique_ptr<ConstraintTerm_t>(new EndEffectorVelocityInFootFrameConstraint_t(
          footIdx, EndEffectorVelocityInFootFrameConstraintSettings_t(), *adComModelPtr_, *adKinematicModelPtr_, options_.recompileLibraries_));

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
    auto _o_EEVelConstraint =
      equalityStateInputConstraintCollection_.template get<EndEffectorVelocityConstraint_t>(footName + "_o_EEVel");
    EndEffectorVelocityConstraintSettings_t _o_eeVelConSettings(1,3);

    // Rolling InFootFrame Velocity constraint for stance legs
    auto _f_EEVelInFootFrameConstraint =
      equalityStateInputConstraintCollection_.template get<EndEffectorVelocityInFootFrameConstraint_t>(footName + "_f_EEVel");

    if (stanceLegs_[footIdx]) {
      // EE velocities in lateral direction (y) in foot frame should be zero.
      EndEffectorVelocityInFootFrameConstraintSettings_t _f_eeVelInFootFrameConSettings(1,3);
      _f_eeVelInFootFrameConSettings.b() << 0;
      _f_eeVelInFootFrameConSettings.A() << 0, 1, 0;
      _f_EEVelInFootFrameConstraint.configure(_f_eeVelInFootFrameConSettings);
      _f_EEVelInFootFrameConstraint.setActivity(true);
      // The upwards velocity (z) in the world frame should be zero too.
      _o_eeVelConSettings.b() << 0;
      _o_eeVelConSettings.A() << 0, 0, 1;
    } else {  // in swing: z-velocity is provided
      _f_EEVelInFootFrameConstraint.setActivity(false);
      _o_eeVelConSettings.b() << -zDirectionRefsPtr_[footIdx]->calculateVelocity(Base::t_);
      _o_eeVelConSettings.A() << 0, 0, 1;
    }
    _o_EEVelConstraint.configure(_o_eeVelConSettings);
    _o_EEVelConstraint.setActivity(true);
  }
}


}  // end of namespace switched_model
