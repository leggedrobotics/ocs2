#include <ocs2_legged_robot_example/constraint/EndEffectorVelocityConstraint.h>
#include <ocs2_legged_robot_example/constraint/FrictionConeConstraint.h>
#include <ocs2_legged_robot_example/constraint/LeggedRobotConstraintAD.h>
#include <ocs2_legged_robot_example/constraint/ZeroForceConstraint.h>

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

namespace ocs2 {
namespace legged_robot {

LeggedRobotConstraintAD::LeggedRobotConstraintAD(std::shared_ptr<const SwitchedModelModeScheduleManager> modeScheduleManagerPtr,
                                                 std::shared_ptr<const SwingTrajectoryPlanner> swingTrajectoryPlannerPtr,
                                                 const PinocchioInterface& pinocchioInterface,
                                                 CentroidalModelPinocchioMapping<ad_scalar_t>& pinocchioMappingAd,
                                                 const ModelSettings& options)
    : Base(),
      modeScheduleManagerPtr_(std::move(modeScheduleManagerPtr)),
      swingTrajectoryPlannerPtr_(std::move(swingTrajectoryPlannerPtr)),
      options_(options),
      equalityStateInputConstraintCollectionPtr_(new ocs2::StateInputConstraintCollection),
      inequalityStateInputConstraintCollectionPtr_(new ocs2::StateInputConstraintCollection) {
  if (!modeScheduleManagerPtr_ || !swingTrajectoryPlannerPtr_) {
    throw std::runtime_error("[LeggedRobotConstraintAD] ModeScheduleManager and SwingTrajectoryPlanner cannot be a nullptr");
  }
  initializeConstraintTerms(pinocchioInterface, pinocchioMappingAd);
}

LeggedRobotConstraintAD::LeggedRobotConstraintAD(const LeggedRobotConstraintAD& rhs)
    : Base(rhs),
      modeScheduleManagerPtr_(rhs.modeScheduleManagerPtr_),
      swingTrajectoryPlannerPtr_(rhs.swingTrajectoryPlannerPtr_),
      options_(rhs.options_),
      equalityStateInputConstraintCollectionPtr_(rhs.equalityStateInputConstraintCollectionPtr_->clone()),
      inequalityStateInputConstraintCollectionPtr_(rhs.inequalityStateInputConstraintCollectionPtr_->clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotConstraintAD* LeggedRobotConstraintAD::clone() const {
  return new LeggedRobotConstraintAD(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotConstraintAD::initializeConstraintTerms(const PinocchioInterface& pinocchioInterface,
                                                        CentroidalModelPinocchioMapping<ad_scalar_t>& pinocchioMappingAd) {
  for (int i = 0; i < FOOT_CONTACTS_NUM_; i++) {
    auto footName = CONTACT_POINTS_NAMES_[i];
    // Friction cone constraint (state-input inequality)
    FrictionConeConstraint::Config newFrictionConeConConfig;
    auto newFrictionConeConstraintPtr = std::unique_ptr<FrictionConeConstraint>(new FrictionConeConstraint(newFrictionConeConConfig, i));

    // Zero force constraint (state-input equality)
    auto zeroForceConstraintPtr = std::unique_ptr<ZeroForceConstraint>(new ZeroForceConstraint(i));

    // Zero velocity constraint (state-input equality)
    PinocchioEndEffectorKinematicsCppAd eeKinematicsAd(pinocchioInterface, pinocchioMappingAd, {footName}, STATE_DIM_, INPUT_DIM_, footName,
                                                       "/tmp/ocs2", options_.recompileLibraries_, true);
    auto eeZeroVelConstraintPtr = std::unique_ptr<EndEffectorVelocityConstraint>(new EndEffectorVelocityConstraint(eeKinematicsAd, 3));
    // Normal velocity constraint (state-input equality)
    auto eeNormalVelConstraintPtr = std::unique_ptr<EndEffectorVelocityConstraint>(new EndEffectorVelocityConstraint(eeKinematicsAd, 1));

    EndEffectorVelocityConstraintSettings eeZeroVelConConfig;
    eeZeroVelConConfig.A.resize(3, 6);
    eeZeroVelConConfig.A << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero();
    eeZeroVelConConfig.A(2, 5) = options_.positionErrorGain_;
    eeZeroVelConConfig.b = Eigen::Vector3d::Zero();
    eeZeroVelConstraintPtr->configure(eeZeroVelConConfig);

    // Inequalities
    if (options_.enforceFrictionConeConstraint_) {
      inequalityStateInputConstraintCollectionPtr_->add(footName + "_FrictionCone", std::move(newFrictionConeConstraintPtr));
    }

    // Equalities
    equalityStateInputConstraintCollectionPtr_->add(footName + "_ZeroForce", std::move(zeroForceConstraintPtr));
    equalityStateInputConstraintCollectionPtr_->add(footName + "_EEZeroVel", std::move(eeZeroVelConstraintPtr));
    equalityStateInputConstraintCollectionPtr_->add(footName + "_EENormalVel", std::move(eeNormalVelConstraintPtr));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotConstraintAD::timeUpdate(scalar_t t) {
  numEventTimes_ = modeScheduleManagerPtr_->getModeSchedule().eventTimes.size();
  stanceLegs_ = modeScheduleManagerPtr_->getContactFlags(t);

  for (int i = 0; i < FOOT_CONTACTS_NUM_; i++) {
    auto footName = CONTACT_POINTS_NAMES_[i];

    if (options_.enforceFrictionConeConstraint_) {
      // Active friction cone constraint for stanceLegs
      inequalityStateInputConstraintCollectionPtr_->get(footName + "_FrictionCone").setActivity(stanceLegs_[i]);
    }

    // Zero forces active for swing legs
    equalityStateInputConstraintCollectionPtr_->get(footName + "_ZeroForce").setActivity(!stanceLegs_[i]);

    // Z-velocity for swing legs
    equalityStateInputConstraintCollectionPtr_->get(footName + "_EEZeroVel").setActivity(stanceLegs_[i]);

    // Active foot placement for stance legs
    auto& eeNormalVelConstraint = equalityStateInputConstraintCollectionPtr_->get<EndEffectorVelocityConstraint>(footName + "_EENormalVel");

    EndEffectorVelocityConstraintSettings eeNormalVelConConfig;
    eeNormalVelConConfig.b.resize(1);
    eeNormalVelConConfig.A.resize(1, 6);
    eeNormalVelConConfig.b << -swingTrajectoryPlannerPtr_->getZvelocityConstraint(i, t) -
                                  options_.positionErrorGain_ * swingTrajectoryPlannerPtr_->getZpositionConstraint(i, t);
    eeNormalVelConConfig.A << 0, 0, 1, 0, 0, options_.positionErrorGain_;
    eeNormalVelConstraint.configure(eeNormalVelConConfig);

    eeNormalVelConstraint.setActivity(!stanceLegs_[i]);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LeggedRobotConstraintAD::stateInputEqualityConstraint(scalar_t t, const vector_t& x, const vector_t& u) {
  timeUpdate(t);
  return equalityStateInputConstraintCollectionPtr_->getValue(t, x, u);
}

vector_t LeggedRobotConstraintAD::inequalityConstraint(scalar_t t, const vector_t& x, const vector_t& u) {
  timeUpdate(t);
  return inequalityStateInputConstraintCollectionPtr_->getValue(t, x, u);
}

VectorFunctionLinearApproximation LeggedRobotConstraintAD::stateInputEqualityConstraintLinearApproximation(scalar_t t, const vector_t& x,
                                                                                                           const vector_t& u) {
  timeUpdate(t);
  return equalityStateInputConstraintCollectionPtr_->getLinearApproximation(t, x, u);
}

VectorFunctionQuadraticApproximation LeggedRobotConstraintAD::inequalityConstraintQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                                                         const vector_t& u) {
  timeUpdate(t);
  return inequalityStateInputConstraintCollectionPtr_->getQuadraticApproximation(t, x, u);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_array_t LeggedRobotConstraintAD::stateInputEqualityConstraintDerivativesEventTimes(scalar_t t, const vector_t& x,
                                                                                          const vector_t& u) {
  // set all to zero
  const size_t numConstraints = equalityStateInputConstraintCollectionPtr_->getNumConstraints(t);
  vector_array_t g1DevArray(numEventTimes_);
  for (auto& g1Dev : g1DevArray) {
    g1Dev.setZero(numConstraints);
  }
  return g1DevArray;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotConstraintAD::setStanceLegs(const contact_flag_t& stanceLegs) {
  stanceLegs_ = stanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotConstraintAD::getStanceLegs(contact_flag_t& stanceLegs) {
  stanceLegs = stanceLegs_;
}

}  // namespace legged_robot
}  // end of namespace ocs2
