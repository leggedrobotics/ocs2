#include "ocs2_switched_model_interface/constraint/ComKinoConstraintImplAd.h"

// Constraints

namespace switched_model {

ComKinoConstraintImplAd::ComKinoConstraintImplAd(const ad_kinematic_model_t& adKinematicModel, const ad_com_model_t& adComModel,
                                                 std::shared_ptr<const logic_rules_t> logicRulesPtr, const ModelSettings& options)
    : Base(adKinematicModel, adComModel, logicRulesPtr, options, false, false) {
  initializeConstraintTerms();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ComKinoConstraintImplAd* ComKinoConstraintImplAd::clone() const {
  return new ComKinoConstraintImplAd(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoConstraintImplAd::initializeConstraintTerms() {
  for (int footIdx = 0; footIdx < NUM_CONTACT_POINTS; footIdx++) {
    auto footName = feetNames[footIdx];

    // Friction cone constraint
    auto frictionCone = std::unique_ptr<ConstraintTerm_t>(new FrictionConeConstraint_t(options_.frictionCoefficient_, 25.0, footIdx));

    // EE force
    auto zeroForceConstraint = std::unique_ptr<ConstraintTerm_t>(new ZeroForceConstraint_t(footIdx));

    // Velocity Constraint
    auto endEffectorVelocityConstraint = std::unique_ptr<ConstraintTerm_t>(new EndEffectorVelocityConstraint_t(
        footIdx, EndEffectorVelocityConstraintSettings_t(), *adComModelPtr_, *adKinematicModelPtr_, options_.recompileLibraries_));

    // Inequalities
    inequalityConstraintCollection_.add(footName + "_FrictionCone", std::move(frictionCone));

    // State input equalities
    equalityStateInputConstraintCollection_.add(footName + "_ZeroForce", std::move(zeroForceConstraint));
    equalityStateInputConstraintCollection_.add(footName + "_EEVel", std::move(endEffectorVelocityConstraint));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoConstraintImplAd::setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) {
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
    auto& EEVelConstraint = equalityStateInputConstraintCollection_.get<EndEffectorVelocityConstraint_t>(footName + "_EEVel");
    EEVelConstraint.setActivity(true);
    EndEffectorVelocityConstraintSettings_t eeVelConSettings(3, 3);
    if (stanceLegs_[footIdx]) {  // in stance: All velocity equal to zero
      eeVelConSettings.b();
      eeVelConSettings.A();
      eeVelConSettings.b() = Eigen::Vector3d::Zero();
      eeVelConSettings.A() = Eigen::Matrix3d::Identity();
    } else {  // in swing: z-velocity is provided
      eeVelConSettings.resize(1, 3);
      eeVelConSettings.b();
      eeVelConSettings.A();
      eeVelConSettings.b() << -zDirectionRefsPtr_[footIdx]->calculateVelocity(Base::t_);
      eeVelConSettings.A() << 0, 0, 1;
    }
    EEVelConstraint.configure(eeVelConSettings);
  }
}

}  // namespace switched_model
