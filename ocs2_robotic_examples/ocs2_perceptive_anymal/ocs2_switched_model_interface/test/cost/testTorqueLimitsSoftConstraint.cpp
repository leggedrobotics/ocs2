//
// Created by rgrandia on 26.07.21.
//

#include <gtest/gtest.h>

#include "ocs2_switched_model_interface/cost/TorqueLimitsSoftConstraint.h"

using namespace switched_model;

TEST(TestTorqueLimits, quadraticApproximation) {
  joint_coordinate_t torqueLimits = joint_coordinate_t::Constant(80.0);
  joint_coordinate_t nominalTorque = joint_coordinate_t::Constant(10.0);
  const scalar_t mu = 0.1;
  const scalar_t delta = 0.05;

  TorqueLimitsSoftConstraint torqueLimitsSoftConstraint(torqueLimits, {mu, delta}, nominalTorque);

  joint_coordinate_t middleTorque = joint_coordinate_t::Zero();
  ocs2::VectorFunctionLinearApproximation torqueDerivatives = VectorFunctionLinearApproximation::Zero(JOINT_COORDINATE_SIZE, STATE_DIM, INPUT_DIM);
  // some random values per joint (with the correct sparsity)
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg){
    const int torqueStartIndex = 3 * leg;
    const int forceStartIndex = 3 * leg;
    const int jointStartStateIndex = 2 * BASE_COORDINATE_SIZE + 3 * leg;

    torqueDerivatives.dfdx.block<3, 3>(torqueStartIndex, jointStartStateIndex).setRandom();
    torqueDerivatives.dfdu.block<3, 3>(torqueStartIndex, forceStartIndex).setRandom();
  }

  auto value = torqueLimitsSoftConstraint.getValue(middleTorque);
  auto quadApprox = torqueLimitsSoftConstraint.getQuadraticApproximation(torqueDerivatives);

  // Gradient is zero in middle of limits
  ASSERT_DOUBLE_EQ(quadApprox.dfdx.norm(), 0.0);
  ASSERT_DOUBLE_EQ(quadApprox.dfdu.norm(), 0.0);

  // Value is consistent
  ASSERT_DOUBLE_EQ(quadApprox.f, value);
}

TEST(TestTorqueLimits, infiniteLimits) {
  joint_coordinate_t torqueLimits = joint_coordinate_t::Constant(1e30);
  joint_coordinate_t nominalTorque = joint_coordinate_t::Constant(10.0);
  const scalar_t mu = 0.1;
  const scalar_t delta = 0.05;

  TorqueLimitsSoftConstraint torqueLimitsSoftConstraint(torqueLimits, {mu, delta}, nominalTorque);

  joint_coordinate_t middleTorque = joint_coordinate_t::Zero();
  ocs2::VectorFunctionLinearApproximation torqueDerivatives = VectorFunctionLinearApproximation::Zero(JOINT_COORDINATE_SIZE, STATE_DIM, INPUT_DIM);
  // some random values per joint (with the correct sparsity)
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg){
    const int torqueStartIndex = 3 * leg;
    const int forceStartIndex = 3 * leg;
    const int jointStartStateIndex = 2 * BASE_COORDINATE_SIZE + 3 * leg;

    torqueDerivatives.dfdx.block<3, 3>(torqueStartIndex, jointStartStateIndex).setRandom();
    torqueDerivatives.dfdu.block<3, 3>(torqueStartIndex, forceStartIndex).setRandom();
  }


  auto quadApprox = torqueLimitsSoftConstraint.getQuadraticApproximation(torqueDerivatives);

  // Gradient and curvature are both almost zero
  ASSERT_LT(quadApprox.dfdx.norm(), 1e-30);
  ASSERT_LT(quadApprox.dfdu.norm(), 1e-30);
  ASSERT_LT(quadApprox.dfdxx.norm(), 1e-30);
  ASSERT_LT(quadApprox.dfdux.norm(), 1e-30);
  ASSERT_LT(quadApprox.dfduu.norm(), 1e-30);
}
