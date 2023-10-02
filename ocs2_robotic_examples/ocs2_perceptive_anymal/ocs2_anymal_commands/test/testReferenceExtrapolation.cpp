//
// Created by rgrandia on 03.11.21.
//

#include <gtest/gtest.h>

#include <ocs2_anymal_commands/ReferenceExtrapolation.h>

#include <ocs2_switched_model_interface/core/Rotations.h>

using namespace switched_model;

TEST(TestReferenceExtrapolation, translationOnflatTerrain) {
  TerrainPlane flatTerrain(vector3_t::Random(), matrix3_t::Identity());

  BaseReferenceHorizon horizon;
  horizon.dt = 0.1;
  horizon.N = 5;

  BaseReferenceState initialState;
  initialState.t0 = 0.42;
  initialState.positionInWorld = vector3_t::Random();
  initialState.eulerXyz = vector3_t{0.0, 0.0, 5.0};

  BaseReferenceCommand command;
  command.headingVelocity = 0.3;
  command.lateralVelocity = -0.1;
  command.yawRate = 0.0;
  command.baseHeight = 0.2;

  const auto baseRef = generateExtrapolatedBaseReference(horizon, initialState, command, flatTerrain);

  const vector3_t initialBasePosition(initialState.positionInWorld.x(), initialState.positionInWorld.y(),
                                                      command.baseHeight + flatTerrain.positionInWorld.z());
  vector3_t velocityCommand(command.headingVelocity, command.lateralVelocity, 0.0);
  rotateInPlaceZ(velocityCommand, initialState.eulerXyz.z());

  for (int k = 0; k < horizon.N; ++k) {
    ASSERT_DOUBLE_EQ(baseRef.time[k], initialState.t0 + k * horizon.dt);
    ASSERT_TRUE(baseRef.positionInWorld[k].isApprox(initialBasePosition + k * horizon.dt * velocityCommand));
    ASSERT_TRUE(baseRef.eulerXyz[k].isApprox(initialState.eulerXyz));
    ASSERT_TRUE(baseRef.linearVelocityInWorld[k].isApprox(velocityCommand));
    ASSERT_TRUE(baseRef.angularVelocityInWorld[k].norm() < 1e-12);
  }
}

TEST(TestReferenceExtrapolation, rotationOnflatTerrain) {
  TerrainPlane flatTerrain(vector3_t::Random(), matrix3_t::Identity());

  BaseReferenceHorizon horizon;
  horizon.dt = 0.1;
  horizon.N = 5;

  BaseReferenceState initialState;
  initialState.t0 = 0.42;
  initialState.positionInWorld = vector3_t::Random();
  initialState.eulerXyz = vector3_t{0.0, 0.0, 5.0};

  BaseReferenceCommand command;
  command.headingVelocity = 0.0;
  command.lateralVelocity = 0.0;
  command.yawRate = 0.4;
  command.baseHeight = 0.2;

  const auto baseRef = generateExtrapolatedBaseReference(horizon, initialState, command, flatTerrain);

  const vector3_t initialBasePosition(initialState.positionInWorld.x(), initialState.positionInWorld.y(),
                                                      command.baseHeight + flatTerrain.positionInWorld.z());
  const auto angularRate = vector3_t(0.0, 0.0, command.yawRate);

  for (int k = 0; k < horizon.N; ++k) {
    ASSERT_DOUBLE_EQ(baseRef.time[k], initialState.t0 + k * horizon.dt);
    ASSERT_TRUE(baseRef.positionInWorld[k].isApprox(initialBasePosition));
    ASSERT_TRUE(baseRef.eulerXyz[k].isApprox(initialState.eulerXyz + k * horizon.dt * angularRate)); // Abuse that eulerXY is zero.
    ASSERT_TRUE(baseRef.linearVelocityInWorld[k].norm() < 1e-12);
    ASSERT_TRUE(baseRef.angularVelocityInWorld[k].isApprox(angularRate));
  }
}