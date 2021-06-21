#pragma once

#include <memory>

#include <ocs2_core/constraint/StateInputConstraint.h>

#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include <ocs2_legged_robot_example/common/definitions.h>

namespace ocs2 {
namespace legged_robot {

struct EndEffectorVelocityConstraintSettings {
  // Constraints are  A * x + b
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
};

class EndEffectorVelocityConstraint final : public ocs2::StateInputConstraint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = ocs2::StateInputConstraint;
  using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;

  explicit EndEffectorVelocityConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics, const size_t numConstraints);

  ~EndEffectorVelocityConstraint() override = default;
  EndEffectorVelocityConstraint* clone() const override;

  void configure(const EndEffectorVelocityConstraintSettings& settings);

  size_t getNumConstraints(scalar_t time) const override;

  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input) const override;

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input) const override;

  EndEffectorKinematics<scalar_t>& getEndEffectorKinematics() { return *endEffectorKinematicsPtr_; }

 private:
  EndEffectorVelocityConstraint(const EndEffectorVelocityConstraint& rhs);

  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;

  const size_t numConstraints_;

  EndEffectorVelocityConstraintSettings settings_;
};
}  // namespace legged_robot
}  // namespace ocs2
