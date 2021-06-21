#include <ocs2_legged_robot_example/constraint/EndEffectorVelocityConstraint.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorVelocityConstraint::EndEffectorVelocityConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                             const size_t numConstraints)
    : BASE(ConstraintOrder::Linear), endEffectorKinematicsPtr_(endEffectorKinematics.clone()), numConstraints_(numConstraints) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorVelocityConstraint::EndEffectorVelocityConstraint(const EndEffectorVelocityConstraint& rhs)
    : BASE(rhs),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      numConstraints_(rhs.numConstraints_),
      settings_(rhs.settings_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorVelocityConstraint* EndEffectorVelocityConstraint::clone() const {
  return new EndEffectorVelocityConstraint(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void EndEffectorVelocityConstraint::configure(const EndEffectorVelocityConstraintSettings& settings) {
  assert(settings.A.rows() == numConstraints_);
  assert(settings.A.cols() == 6);
  assert(settings.b.rows() == numConstraints_);
  settings_ = settings;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t EndEffectorVelocityConstraint::getNumConstraints(scalar_t time) const {
  return numConstraints_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t EndEffectorVelocityConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input) const {
  // Compute constraints
  const vector3_t eeVelocityWorld = endEffectorKinematicsPtr_->getVelocity(state, input)[0];
  const vector3_t eePositionWorld = endEffectorKinematicsPtr_->getPosition(state)[0];

  return settings_.A * (vector_t(6) << eeVelocityWorld, eePositionWorld).finished() + settings_.b;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation EndEffectorVelocityConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                        const vector_t& input) const {
  // Convert to output format
  VectorFunctionLinearApproximation linearApproximation(getNumConstraints(time), STATE_DIM_, INPUT_DIM_);
  const auto velocityLinearApproximation = endEffectorKinematicsPtr_->getVelocityLinearApproximation(state, input)[0];
  const auto positionLinearApproximation = endEffectorKinematicsPtr_->getPositionLinearApproximation(state)[0];

  linearApproximation.f =
      settings_.A * (vector_t(6) << velocityLinearApproximation.f, positionLinearApproximation.f).finished() + settings_.b;
  linearApproximation.dfdx =
      settings_.A * (matrix_t(6, STATE_DIM_) << velocityLinearApproximation.dfdx, positionLinearApproximation.dfdx).finished();
  linearApproximation.dfdu =
      settings_.A * (matrix_t(6, INPUT_DIM_) << velocityLinearApproximation.dfdu, matrix_t::Zero(3, input.size())).finished();

  return linearApproximation;
}

}  // namespace legged_robot
}  // namespace ocs2
