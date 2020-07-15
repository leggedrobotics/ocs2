#include "ocs2_switched_model_interface/constraint/ComKinoConstraintBaseAd.h"

// Constraints
#include "ocs2_switched_model_interface/constraint/EndEffectorVelocityConstraint.h"
#include "ocs2_switched_model_interface/constraint/FootNormalContraint.h"
#include "ocs2_switched_model_interface/constraint/FrictionConeConstraint.h"
#include "ocs2_switched_model_interface/constraint/ZeroForceConstraint.h"

namespace switched_model {

ComKinoConstraintBaseAd::ComKinoConstraintBaseAd(const ad_kinematic_model_t& adKinematicModel, const ad_com_model_t& adComModel,
                                                 std::shared_ptr<const SwitchedModelModeScheduleManager> modeScheduleManagerPtr,
                                                 std::shared_ptr<const SwingTrajectoryPlanner> swingTrajectoryPlannerPtr,
                                                 ModelSettings options)
    : adKinematicModelPtr_(adKinematicModel.clone()),
      adComModelPtr_(adComModel.clone()),
      modeScheduleManagerPtr_(std::move(modeScheduleManagerPtr)),
      swingTrajectoryPlannerPtr_(std::move(swingTrajectoryPlannerPtr)),
      options_(std::move(options)) {
  if (!modeScheduleManagerPtr_ || !swingTrajectoryPlannerPtr_) {
    throw std::runtime_error("[ComKinoConstraintBaseAD] ModeScheduleManager and SwingTrajectoryPlanner cannot be a nullptr");
  }
  initializeConstraintTerms();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ComKinoConstraintBaseAd::ComKinoConstraintBaseAd(const ComKinoConstraintBaseAd& rhs)
    : Base(rhs),
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
void ComKinoConstraintBaseAd::timeUpdate(scalar_t t) {
  numEventTimes_ = modeScheduleManagerPtr_->getModeSchedule().eventTimes.size();
  stanceLegs_ = modeScheduleManagerPtr_->getContactFlags(t);

  for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
    auto footName = feetNames[i];

    // Active friction cone constraint for stanceLegs
    inequalityConstraintCollection_.get(footName + "_FrictionCone").setActivity(stanceLegs_[i]);

    // Zero forces active for swing legs
    equalityStateInputConstraintCollection_.get(footName + "_ZeroForce").setActivity(!stanceLegs_[i]);

    // Foot normal constraint always active
    auto& EENormalConstraint = equalityStateInputConstraintCollection_.get<FootNormalConstraint>(footName + "_EENormal");
    EENormalConstraint.setActivity(true);
    const auto normalDirectionConstraint = swingTrajectoryPlannerPtr_->getNormalDirectionConstraint(i, t);
    EENormalConstraint.configure(normalDirectionConstraint);

    // Foot tangential constraints only for stanceLegs
    auto& EEVelConstraint = equalityStateInputConstraintCollection_.get<EndEffectorVelocityConstraint>(footName + "_EEVel");
    EEVelConstraint.setActivity(stanceLegs_[i]);
    if (stanceLegs_[i]) {
      EndEffectorVelocityConstraintSettings eeVelConSettings;
      eeVelConSettings.A = tangentialBasisFromSurfaceNormal(normalDirectionConstraint.velocityMatrix);
      eeVelConSettings.b = Eigen::Vector2d::Zero();
      EEVelConstraint.configure(eeVelConSettings);
    }
  }
}

vector_t ComKinoConstraintBaseAd::stateInputEqualityConstraint(scalar_t t, const vector_t& x, const vector_t& u) {
  timeUpdate(t);
  return equalityStateInputConstraintCollection_.getValueAsVector(t, x, u);
}

vector_t ComKinoConstraintBaseAd::inequalityConstraint(scalar_t t, const vector_t& x, const vector_t& u) {
  timeUpdate(t);
  return inequalityConstraintCollection_.getValueAsVector(t, x, u);
}

VectorFunctionLinearApproximation ComKinoConstraintBaseAd::stateInputEqualityConstraintLinearApproximation(scalar_t t, const vector_t& x,
                                                                                                           const vector_t& u) {
  timeUpdate(t);
  const size_t numConstraints = equalityStateInputConstraintCollection_.getNumConstraints(t);
  const auto constraintApproximation = equalityStateInputConstraintCollection_.getLinearApproximationAsMatrices(t, x, u);

  VectorFunctionLinearApproximation g;
  g.f = constraintApproximation.constraintValues.head(numConstraints);
  g.dfdx = constraintApproximation.derivativeState.topRows(numConstraints);
  g.dfdu = constraintApproximation.derivativeInput.topRows(numConstraints);
  return g;
}

VectorFunctionQuadraticApproximation ComKinoConstraintBaseAd::inequalityConstraintQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                                                         const vector_t& u) {
  timeUpdate(t);
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
  vector_array_t g1DevArray(numEventTimes_);
  for (auto& g1Dev : g1DevArray) {
    g1Dev.setZero(numConstraints);
  }
  return g1DevArray;
}

}  // end of namespace switched_model
