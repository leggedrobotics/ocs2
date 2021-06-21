#include <ocs2_legged_robot_example/constraint/ZeroForceConstraint.h>

namespace ocs2 {
namespace legged_robot {

ZeroForceConstraint::ZeroForceConstraint(int contactPointNumber) : BASE(ConstraintOrder::Linear), contactPointNumber_(contactPointNumber) {}

ZeroForceConstraint* ZeroForceConstraint::clone() const {
  return new ZeroForceConstraint(*this);
}

size_t ZeroForceConstraint::getNumConstraints(scalar_t time) const {
  return 3;
};

vector_t ZeroForceConstraint::getValue(scalar_t time, const ocs2::vector_t& state, const ocs2::vector_t& input) const {
  const scalar_t Fx = input(3 * contactPointNumber_ + 0);
  const scalar_t Fy = input(3 * contactPointNumber_ + 1);
  const scalar_t Fz = input(3 * contactPointNumber_ + 2);

  vector_t constraintValues(getNumConstraints(time));
  constraintValues[0] = Fx;
  constraintValues[1] = Fy;
  constraintValues[2] = Fz;
  return constraintValues;
};

VectorFunctionLinearApproximation ZeroForceConstraint::getLinearApproximation(scalar_t time, const ocs2::vector_t& state,
                                                                              const ocs2::vector_t& input) const {
  VectorFunctionLinearApproximation linearApproximation =
      VectorFunctionLinearApproximation::Zero(getNumConstraints(time), STATE_DIM_, INPUT_DIM_);
  for (int i = 0; i < 3; i++) {
    linearApproximation.f[i] = input(3 * contactPointNumber_ + i);
    linearApproximation.dfdu(i, 3 * contactPointNumber_ + i) = 1.0;
  }

  return linearApproximation;
}

}  // namespace legged_robot
}  // namespace ocs2
