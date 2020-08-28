#include "ocs2_switched_model_interface/constraint/ComKinoConstraintBaseAd.h"

// Constraints
#include "ocs2_switched_model_interface/constraint/EndEffectorVelocityConstraint.h"
#include "ocs2_switched_model_interface/constraint/FootNormalConstraint.h"
#include "ocs2_switched_model_interface/constraint/FrictionConeConstraint.h"
#include "ocs2_switched_model_interface/constraint/ZeroForceConstraint.h"

namespace switched_model {

ComKinoConstraintBaseAd::ComKinoConstraintBaseAd(const ad_kinematic_model_t& adKinematicModel, const ad_com_model_t& adComModel,
                                                 const SwitchedModelModeScheduleManager& modeScheduleManager,
                                                 const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings options)
    : adKinematicModelPtr_(adKinematicModel.clone()),
      adComModelPtr_(adComModel.clone()),
      modeScheduleManagerPtr_(&modeScheduleManager),
      swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner),
      options_(std::move(options)) {
  initializeConstraintTerms();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ComKinoConstraintBaseAd::ComKinoConstraintBaseAd(const ComKinoConstraintBaseAd& rhs)
    : ocs2::ConstraintBase(rhs),
      adKinematicModelPtr_(rhs.adKinematicModelPtr_->clone()),
      adComModelPtr_(rhs.adComModelPtr_->clone()),
      modeScheduleManagerPtr_(rhs.modeScheduleManagerPtr_),
      swingTrajectoryPlannerPtr_(rhs.swingTrajectoryPlannerPtr_),
      options_(rhs.options_),
      inequalityConstraintCollection_(rhs.inequalityConstraintCollection_),
      equalityStateInputConstraintCollection_(rhs.equalityStateInputConstraintCollection_) {}

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
  using ConstraintTerm_t = ocs2::ConstraintTerm<STATE_DIM, INPUT_DIM>;

  for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
    auto footName = feetNames[i];

    // Friction cone constraint
    auto frictionCone = std::unique_ptr<ConstraintTerm_t>(new FrictionConeConstraint(options_.frictionCoefficient_, 25.0, i));

    // EE force
    auto zeroForceConstraint = std::unique_ptr<ConstraintTerm_t>(new ZeroForceConstraint(i));

    // Velocity Constraint
    auto footNormalConstraint = std::unique_ptr<ConstraintTerm_t>(
        new FootNormalConstraint(i, FootNormalConstraintMatrix(), *adComModelPtr_, *adKinematicModelPtr_, options_.recompileLibraries_));
    auto endEffectorVelocityConstraint = std::unique_ptr<ConstraintTerm_t>(new EndEffectorVelocityConstraint(
        i, EndEffectorVelocityConstraintSettings(), *adComModelPtr_, *adKinematicModelPtr_, options_.recompileLibraries_));

    // Inequalities
    inequalityConstraintCollection_.add(footName + "_FrictionCone", std::move(frictionCone));

    // State input equalities
    equalityStateInputConstraintCollection_.add(footName + "_ZeroForce", std::move(zeroForceConstraint));
    equalityStateInputConstraintCollection_.add(footName + "_EENormal", std::move(footNormalConstraint));
    equalityStateInputConstraintCollection_.add(footName + "_EEVel", std::move(endEffectorVelocityConstraint));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoConstraintBaseAd::updateStateInputEqualityConstraints(scalar_t t) {
  for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
    const auto footName = feetNames[i];
    const auto& footPhase = swingTrajectoryPlannerPtr_->getFootPhase(i, t);
    const bool inContact = footPhase.contactFlag();

    // Zero forces active for swing legs
    equalityStateInputConstraintCollection_.get(footName + "_ZeroForce").setActivity(!inContact);

    // Foot normal constraint always active
    auto& EENormalConstraint = equalityStateInputConstraintCollection_.get<FootNormalConstraint>(footName + "_EENormal");
    EENormalConstraint.setActivity(true);
    EENormalConstraint.configure(footPhase.getFootNormalConstraintInWorldFrame(t));

    // Foot tangential constraints only for stanceLegs
    auto& EEVelConstraint = equalityStateInputConstraintCollection_.get<EndEffectorVelocityConstraint>(footName + "_EEVel");
    EEVelConstraint.setActivity(inContact);
    if (inContact) {
      EndEffectorVelocityConstraintSettings eeVelConSettings;
      eeVelConSettings.A = tangentialBasisFromSurfaceNormal(footPhase.normalDirectionInWorldFrame(t));
      eeVelConSettings.b = Eigen::Vector2d::Zero();
      EEVelConstraint.configure(eeVelConSettings);
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoConstraintBaseAd::updateInequalityConstraints(scalar_t t) {
  for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
    const auto footName = feetNames[i];
    const auto& footPhase = swingTrajectoryPlannerPtr_->getFootPhase(i, t);
    const bool inContact = footPhase.contactFlag();

    // Active friction cone constraint for stanceLegs
    auto& frictionConeConstraint = inequalityConstraintCollection_.get<FrictionConeConstraint>(footName + "_FrictionCone");
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
  return equalityStateInputConstraintCollection_.getValueAsVector(t, x, u);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ComKinoConstraintBaseAd::inequalityConstraint(scalar_t t, const vector_t& x, const vector_t& u) {
  updateInequalityConstraints(t);
  return inequalityConstraintCollection_.getValueAsVector(t, x, u);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ComKinoConstraintBaseAd::stateInputEqualityConstraintLinearApproximation(scalar_t t, const vector_t& x,
                                                                                                           const vector_t& u) {
  updateStateInputEqualityConstraints(t);
  const size_t numConstraints = equalityStateInputConstraintCollection_.getNumConstraints(t);
  const auto constraintApproximation = equalityStateInputConstraintCollection_.getLinearApproximationAsMatrices(t, x, u);

  VectorFunctionLinearApproximation g;
  g.f = constraintApproximation.constraintValues.head(numConstraints);
  g.dfdx = constraintApproximation.derivativeState.topRows(numConstraints);
  g.dfdu = constraintApproximation.derivativeInput.topRows(numConstraints);
  return g;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation ComKinoConstraintBaseAd::inequalityConstraintQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                                                         const vector_t& u) {
  updateInequalityConstraints(t);
  const auto constraintApproximation = inequalityConstraintCollection_.getQuadraticApproximation(t, x, u);
  const size_t numConstraints = constraintApproximation.constraintValues.size();

  VectorFunctionQuadraticApproximation h;
  h.f.resize(numConstraints);
  h.dfdx.resize(numConstraints, x.rows());
  h.dfdu.resize(numConstraints, u.rows());
  h.dfdxx.resize(numConstraints);
  h.dfduu.resize(numConstraints);
  h.dfdux.resize(numConstraints);
  for (size_t i = 0; i < numConstraints; i++) {
    h.f(i) = constraintApproximation.constraintValues[i];
    h.dfdx.row(i) = constraintApproximation.derivativeState[i];
    h.dfdu.row(i) = constraintApproximation.derivativeInput[i];
    h.dfdxx[i] = constraintApproximation.secondDerivativesState[i];
    h.dfduu[i] = constraintApproximation.secondDerivativesInput[i];
    h.dfdux[i] = constraintApproximation.derivativesInputState[i];
  }
  return h;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_array_t ComKinoConstraintBaseAd::stateInputEqualityConstraintDerivativesEventTimes(scalar_t t, const vector_t& x,
                                                                                          const vector_t& u) {
  // set all to zero
  const size_t numConstraints = equalityStateInputConstraintCollection_.getNumConstraints(t);
  vector_array_t g1DevArray(modeScheduleManagerPtr_->getModeSchedule().eventTimes.size());
  for (auto& g1Dev : g1DevArray) {
    g1Dev.setZero(numConstraints);
  }
  return g1DevArray;
}

}  // end of namespace switched_model
