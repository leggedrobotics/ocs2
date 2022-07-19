//
// Created by rgrandia on 29.04.20.
//

#include "ocs2_switched_model_interface/constraint/FootNormalConstraint.h"

#include "ocs2_switched_model_interface/core/SwitchedModelPrecomputation.h"

namespace switched_model {

namespace {

/**
 * Linear constraint A_p * p_world + A_v * v_world + b = 0
 */
struct FootNormalConstraintMatrix {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<scalar_t, 1, 3> positionMatrix = Eigen::Matrix<scalar_t, 1, 3>::Zero();
  Eigen::Matrix<scalar_t, 1, 3> velocityMatrix = Eigen::Matrix<scalar_t, 1, 3>::Zero();
  scalar_t constant = 0.0;
};

/**
 * Constructs a velocity constraint in surface normal direction :
 *  ==> v_foot = v_ff - kp * (p_foot - p_des)
 *  ==> (n')* v_foot + (kp* n')* p_foot - (n') * (v_ff + kp* p_des) = 0
 *
 *  Gives us
 *  ==> A_p * p_world + A_v * v_world + b = 0
 *  A_p = kp * n'
 *  A_v = n'
 *  b = (n') * (v_ff + kp* p_des)
 */
FootNormalConstraintMatrix computeFootNormalConstraint(const vector3_t& surfaceNormalInWorld, const vector3_t& feedforwardVelocityInWorld,
                                                       const vector3_t& feedforwardPositionInWorld, scalar_t positionGain) {
  FootNormalConstraintMatrix footNormalConstraint;
  footNormalConstraint.velocityMatrix = surfaceNormalInWorld.transpose();
  footNormalConstraint.positionMatrix = positionGain * surfaceNormalInWorld.transpose();
  footNormalConstraint.constant = -surfaceNormalInWorld.dot(feedforwardVelocityInWorld + positionGain * feedforwardPositionInWorld);
  return footNormalConstraint;
}
}  // namespace

FootNormalConstraint::FootNormalConstraint(int legNumber, scalar_t positionGain)
    : ocs2::StateInputConstraint(ocs2::ConstraintOrder::Linear), legNumber_(legNumber), positionGain_(positionGain) {}

FootNormalConstraint::FootNormalConstraint(const FootNormalConstraint& rhs)
    : ocs2::StateInputConstraint(rhs), legNumber_(rhs.legNumber_), positionGain_(rhs.positionGain_) {}

FootNormalConstraint* FootNormalConstraint::clone() const {
  return new FootNormalConstraint(*this);
}

vector_t FootNormalConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                        const ocs2::PreComputation& preComp) const {
  const auto& switchedModelPreComp = ocs2::cast<SwitchedModelPreComputation>(preComp);
  const auto& motionReference = switchedModelPreComp.getMotionReference();
  const auto& normalConstraint =
      computeFootNormalConstraint(switchedModelPreComp.getSurfaceNormalInOriginFrame(legNumber_), motionReference.footVelocity[legNumber_],
                                  motionReference.footPosition[legNumber_], positionGain_);
  const auto& o_footPosition = switchedModelPreComp.footPositionInOriginFrame(legNumber_);
  const auto& o_footVelocity = switchedModelPreComp.footVelocityInOriginFrame(legNumber_);

  vector_t h(1);
  h[0] =
      normalConstraint.positionMatrix.dot(o_footPosition) + normalConstraint.velocityMatrix.dot(o_footVelocity) + normalConstraint.constant;
  return h;
}

VectorFunctionLinearApproximation FootNormalConstraint::getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                               const ocs2::PreComputation& preComp) const {
  const auto& switchedModelPreComp = ocs2::cast<SwitchedModelPreComputation>(preComp);
  const auto& motionReference = switchedModelPreComp.getMotionReference();
  const auto& normalConstraint =
      computeFootNormalConstraint(switchedModelPreComp.getSurfaceNormalInOriginFrame(legNumber_), motionReference.footVelocity[legNumber_],
                                  motionReference.footPosition[legNumber_], positionGain_);
  const auto& o_footPosition = switchedModelPreComp.footPositionInOriginFrame(legNumber_);
  const auto& o_footPositionDerivative = switchedModelPreComp.footPositionInOriginFrameStateDerivative(legNumber_);
  const auto& o_footVelocity = switchedModelPreComp.footVelocityInOriginFrame(legNumber_);
  const auto& o_footVelocityDerivative = switchedModelPreComp.footVelocityInOriginFrameDerivative(legNumber_);

  VectorFunctionLinearApproximation constraint;
  // Constant
  constraint.f.resize(1);
  constraint.f[0] =
      normalConstraint.positionMatrix.dot(o_footPosition) + normalConstraint.velocityMatrix.dot(o_footVelocity) + normalConstraint.constant;

  // State derivative
  constraint.dfdx.noalias() = normalConstraint.positionMatrix * o_footPositionDerivative;
  constraint.dfdx.noalias() += normalConstraint.velocityMatrix * o_footVelocityDerivative.dfdx;

  // Input derivative
  constraint.dfdu.noalias() = normalConstraint.velocityMatrix * o_footVelocityDerivative.dfdu;
  return constraint;
}

}  // namespace switched_model
