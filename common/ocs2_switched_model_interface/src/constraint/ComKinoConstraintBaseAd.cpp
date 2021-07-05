#include "ocs2_switched_model_interface/constraint/ComKinoConstraintBaseAd.h"

namespace switched_model {

ComKinoConstraintBaseAd::ComKinoConstraintBaseAd(const ad_kinematic_model_t& adKinematicModel, const ad_com_model_t& adComModel,
                                                 const SwitchedModelModeScheduleManager& modeScheduleManager,
                                                 const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings options)
    : equalityStateInputConstraintCollectionPtr_(new ocs2::StateInputConstraintCollection),
      inequalityConstraintsCollectionPtr_(new ocs2::StateInputConstraintCollection),
      adKinematicModelPtr_(adKinematicModel.clone()),
      adComModelPtr_(adComModel.clone()),
      options_(std::move(options)),
      modeScheduleManagerPtr_(&modeScheduleManager),
      swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner) {
  initializeConstraintTerms();
  collectConstraintPointers();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ComKinoConstraintBaseAd::ComKinoConstraintBaseAd(const ComKinoConstraintBaseAd& rhs)
    : ocs2::ConstraintBase(rhs),
      equalityStateInputConstraintCollectionPtr_(rhs.equalityStateInputConstraintCollectionPtr_->clone()),
      inequalityConstraintsCollectionPtr_(rhs.inequalityConstraintsCollectionPtr_->clone()),
      adKinematicModelPtr_(rhs.adKinematicModelPtr_->clone()),
      adComModelPtr_(rhs.adComModelPtr_->clone()),
      options_(rhs.options_),
      modeScheduleManagerPtr_(rhs.modeScheduleManagerPtr_),
      swingTrajectoryPlannerPtr_(rhs.swingTrajectoryPlannerPtr_) {
  collectConstraintPointers();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ComKinoConstraintBaseAd* ComKinoConstraintBaseAd::clone() const {
  return new ComKinoConstraintBaseAd(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoConstraintBaseAd::initializeConstraintTerms() {
  for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
    auto footName = feetNames[i];

    // Friction cone constraint
    FrictionConeConstraint::Config frictionConfig(options_.frictionCoefficient_);
    std::unique_ptr<FrictionConeConstraint> frictionCone(new FrictionConeConstraint(std::move(frictionConfig), i));

    // Velocity Constraint
    auto footNormalConstraint = std::unique_ptr<ocs2::StateInputConstraint>(
        new FootNormalConstraint(i, FootNormalConstraintMatrix(), *adComModelPtr_, *adKinematicModelPtr_, options_.recompileLibraries_));
    auto endEffectorVelocityConstraint = std::unique_ptr<ocs2::StateInputConstraint>(new EndEffectorVelocityConstraint(
        i, EndEffectorVelocityConstraintSettings(), *adComModelPtr_, *adKinematicModelPtr_, options_.recompileLibraries_));

    // Inequalities
    inequalityConstraintsCollectionPtr_->add(footName + "_FrictionCone", std::move(frictionCone));

    // State input equalities
    equalityStateInputConstraintCollectionPtr_->add(footName + "_EENormal", std::move(footNormalConstraint));
    equalityStateInputConstraintCollectionPtr_->add(footName + "_EEVel", std::move(endEffectorVelocityConstraint));
  }

  // EE force
  auto zeroForceConstraint = std::unique_ptr<ocs2::StateInputConstraint>(new ZeroForceConstraint());
  equalityStateInputConstraintCollectionPtr_->add("ZeroForce", std::move(zeroForceConstraint));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoConstraintBaseAd::collectConstraintPointers() {
  for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
    auto footName = feetNames[i];

    // Inequalities
    frictionConeConstraints_[i] = &inequalityConstraintsCollectionPtr_->get<FrictionConeConstraint>(footName + "_FrictionCone");

    // State input equalities
    eeNormalConstraints_[i] = &equalityStateInputConstraintCollectionPtr_->get<FootNormalConstraint>(footName + "_EENormal");
    eeVelConstraints_[i] = &equalityStateInputConstraintCollectionPtr_->get<EndEffectorVelocityConstraint>(footName + "_EEVel");
  }

  zeroForceConstraints_ = &equalityStateInputConstraintCollectionPtr_->get<ZeroForceConstraint>("ZeroForce");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoConstraintBaseAd::updateStateInputEqualityConstraints(scalar_t t) {
  for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
    const auto& footPhase = swingTrajectoryPlannerPtr_->getFootPhase(i, t);
    const bool inContact = footPhase.contactFlag();

    // Foot normal constraint always active
    auto& EENormalConstraint = *eeNormalConstraints_[i];
    EENormalConstraint.setActivity(true);
    EENormalConstraint.configure(footPhase.getFootNormalConstraintInWorldFrame(t));

    // Foot tangential constraints only for stanceLegs
    auto& EEVelConstraint = *eeVelConstraints_[i];
    EEVelConstraint.setActivity(inContact);
    if (inContact) {
      EndEffectorVelocityConstraintSettings eeVelConSettings;
      eeVelConSettings.A = tangentialBasisFromSurfaceNormal(footPhase.normalDirectionInWorldFrame(t));
      eeVelConSettings.b = Eigen::Vector2d::Zero();
      EEVelConstraint.configure(eeVelConSettings);
    }
  }

  zeroForceConstraints_->setContactFlags(modeScheduleManagerPtr_->getContactFlags(t));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoConstraintBaseAd::updateInequalityConstraints(scalar_t t) {
  for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
    const auto& footPhase = swingTrajectoryPlannerPtr_->getFootPhase(i, t);
    const bool inContact = footPhase.contactFlag();

    // Active friction cone constraint for stanceLegs
    auto& frictionConeConstraint = *frictionConeConstraints_[i];
    frictionConeConstraint.setActivity(inContact);
    if (inContact) {
      frictionConeConstraint.setSurfaceNormalInWorld(footPhase.normalDirectionInWorldFrame(t));
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ComKinoConstraintBaseAd::stateInputEqualityConstraint(scalar_t t, const vector_t& x, const vector_t& u) {
  updateStateInputEqualityConstraints(t);
  return equalityStateInputConstraintCollectionPtr_->getValue(t, x, u);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ComKinoConstraintBaseAd::inequalityConstraint(scalar_t t, const vector_t& x, const vector_t& u) {
  updateInequalityConstraints(t);
  return inequalityConstraintsCollectionPtr_->getValue(t, x, u);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ComKinoConstraintBaseAd::stateInputEqualityConstraintLinearApproximation(scalar_t t, const vector_t& x,
                                                                                                           const vector_t& u) {
  updateStateInputEqualityConstraints(t);
  return equalityStateInputConstraintCollectionPtr_->getLinearApproximation(t, x, u);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation ComKinoConstraintBaseAd::inequalityConstraintQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                                                         const vector_t& u) {
  updateInequalityConstraints(t);
  return inequalityConstraintsCollectionPtr_->getQuadraticApproximation(t, x, u);
}

}  // end of namespace switched_model
