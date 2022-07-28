//
// Created by rgrandia on 02.03.20.
//

#include "ocs2_anymal_commands/ReferenceExtrapolation.h"

#include <ocs2_anymal_commands/TerrainAdaptation.h>
#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_switched_model_interface/terrain/PlaneFitting.h>
#include <grid_map_filters_rsl/lookup.hpp>

namespace switched_model {

namespace {
void addVelocitiesFromFiniteDifference(BaseReferenceTrajectory& baseRef) {
  auto N = baseRef.time.size();
  if (N <= 1) {
    return;
  }

  baseRef.linearVelocityInWorld.clear();
  baseRef.angularVelocityInWorld.clear();
  baseRef.linearVelocityInWorld.reserve(N);
  baseRef.angularVelocityInWorld.reserve(N);

  for (int k = 0; (k + 1) < baseRef.time.size(); ++k) {
    auto dt = baseRef.time[k + 1] - baseRef.time[k];
    baseRef.angularVelocityInWorld.push_back(rotationErrorInWorldEulerXYZ(baseRef.eulerXyz[k + 1], baseRef.eulerXyz[k]) / dt);
    baseRef.linearVelocityInWorld.push_back((baseRef.positionInWorld[k + 1] - baseRef.positionInWorld[k]) / dt);
  }

  auto dt = baseRef.time[N - 1] - baseRef.time[N - 2];
  baseRef.angularVelocityInWorld.push_back(rotationErrorInWorldEulerXYZ(baseRef.eulerXyz[N - 1], baseRef.eulerXyz[N - 2]) / dt);
  baseRef.linearVelocityInWorld.push_back((baseRef.positionInWorld[N - 1] - baseRef.positionInWorld[N - 2]) / dt);
}
}  // namespace

Eigen::Vector3d velocityCommandInWorld(double headingVelocity, double lateralVelocity, double yaw) {
  Eigen::Vector3d headingReference = Eigen::Vector3d::UnitX();
  Eigen::Vector3d lateralHeadingRef = Eigen::Vector3d::UnitY();
  rotateInPlaceZ(headingReference, yaw);
  rotateInPlaceZ(lateralHeadingRef, yaw);

  return headingVelocity * headingReference + lateralVelocity * lateralHeadingRef;
}

Eigen::Vector3d velocityCommandInWorld(double headingVelocity, double lateralVelocity, const Eigen::Vector3d& eulerXyz) {
  // Project to horizontal plane while preserving heading.
  const auto yaw = alignDesiredOrientationToTerrain(eulerXyz, TerrainPlane()).z();
  return velocityCommandInWorld(headingVelocity, lateralVelocity, yaw);
}

BasePoseReferenceTrajectory generate2DExtrapolatedBaseReference(const BaseReferenceHorizon& horizon, const BaseReferenceState& initialState,
                                                                const BaseReferenceCommand& command) {
  const double dt = horizon.dt;

  BasePoseReferenceTrajectory baseRef;
  baseRef.time.reserve(horizon.N);
  baseRef.eulerXyz.reserve(horizon.N);
  baseRef.positionInWorld.reserve(horizon.N);

  baseRef.time.push_back(initialState.t0);

  // Project position and orientation to horizontal plane
  baseRef.positionInWorld.emplace_back(initialState.positionInWorld.x(), initialState.positionInWorld.y(), 0.0);
  baseRef.eulerXyz.emplace_back(alignDesiredOrientationToTerrain(initialState.eulerXyz, TerrainPlane()));

  for (int k = 1; k < horizon.N; ++k) {
    baseRef.time.push_back(baseRef.time.back() + dt);

    // Express command in world
    const auto commandedLinearVelocityInWorld =
        velocityCommandInWorld(command.headingVelocity, command.lateralVelocity, baseRef.eulerXyz.back().z() + 0.5 * dt * command.yawRate);

    // Advance position
    baseRef.positionInWorld.push_back(baseRef.positionInWorld.back());
    baseRef.positionInWorld.back() += dt * commandedLinearVelocityInWorld;
    baseRef.positionInWorld.back().z() = 0;

    // Advance orientation
    baseRef.eulerXyz.push_back(baseRef.eulerXyz.back());
    baseRef.eulerXyz.back().z() += dt * command.yawRate;
  }

  return baseRef;
}

BaseReferenceTrajectory generateExtrapolatedBaseReference(const BaseReferenceHorizon& horizon, const BaseReferenceState& initialState,
                                                          const BaseReferenceCommand& command, const TerrainPlane& projectedHeadingFrame) {
  auto reference2d = generate2DExtrapolatedBaseReference(horizon, initialState, command);

  BaseReferenceTrajectory baseRef;
  baseRef.time = std::move(reference2d.time);
  baseRef.eulerXyz.reserve(horizon.N);
  baseRef.positionInWorld.reserve(horizon.N);

  // Adapt poses
  for (int k = 0; k < horizon.N; ++k) {
    baseRef.positionInWorld.push_back(
        adaptDesiredPositionHeightToTerrain(reference2d.positionInWorld[k], projectedHeadingFrame, command.baseHeight));
    baseRef.eulerXyz.emplace_back(alignDesiredOrientationToTerrain(reference2d.eulerXyz[k], projectedHeadingFrame));
  }

  addVelocitiesFromFiniteDifference(baseRef);
  return baseRef;
}

BaseReferenceTrajectory generateExtrapolatedBaseReference(const BaseReferenceHorizon& horizon, const BaseReferenceState& initialState,
                                                          const BaseReferenceCommand& command, const grid_map::GridMap& gridMap,
                                                          double nominalStanceWidthInHeading, double nominalStanceWidthLateral) {
  const auto& baseReferenceLayer = gridMap.get("smooth_planar");

  // Helper to get a projected heading frame derived from the terrain.
  auto getLocalHeadingFrame = [&](const vector3_t& basePosition, const vector3_t& eulerXYZ) {
    // eulerXYZ only has Z component, due to 2d generation step.
    vector3_t lfOffset(nominalStanceWidthInHeading / 2.0, nominalStanceWidthLateral / 2.0, 0.0);
    vector3_t rfOffset(nominalStanceWidthInHeading / 2.0, -nominalStanceWidthLateral / 2.0, 0.0);
    vector3_t lhOffset(-nominalStanceWidthInHeading / 2.0, nominalStanceWidthLateral / 2.0, 0.0);
    vector3_t rhOffset(-nominalStanceWidthInHeading / 2.0, -nominalStanceWidthLateral / 2.0, 0.0);
    // Rotate from heading to world frame
    rotateInPlaceZ(lfOffset, eulerXYZ.z());
    rotateInPlaceZ(rfOffset, eulerXYZ.z());
    rotateInPlaceZ(lhOffset, eulerXYZ.z());
    rotateInPlaceZ(rhOffset, eulerXYZ.z());
    // shift by base center
    lfOffset += basePosition;
    rfOffset += basePosition;
    lhOffset += basePosition;
    rhOffset += basePosition;

    auto interp = [&](const vector3_t& offset) {
      auto projection = grid_map::lookup::projectToMapWithMargin(gridMap, grid_map::Position(offset.x(), offset.y()));

      try {
        auto z = gridMap.atPosition("smooth_planar", projection, grid_map::InterpolationMethods::INTER_NEAREST);
        return vector3_t(offset.x(), offset.y(), z);
      } catch (std::out_of_range& e) {
        double interp = gridMap.getResolution() / (projection - gridMap.getPosition()).norm();
        projection = (1.0 - interp) * projection + interp * gridMap.getPosition();
        auto z = gridMap.atPosition("smooth_planar", projection, grid_map::InterpolationMethods::INTER_NEAREST);
        return vector3_t(offset.x(), offset.y(), z);
      }
    };

    vector3_t lfVerticalProjection = interp(lfOffset);
    vector3_t rfVerticalProjection = interp(rfOffset);
    vector3_t lhVerticalProjection = interp(lhOffset);
    vector3_t rhVerticalProjection = interp(rhOffset);

    const auto normalAndPosition = estimatePlane({lfVerticalProjection, rfVerticalProjection, lhVerticalProjection, rhVerticalProjection});

    TerrainPlane terrainPlane(normalAndPosition.position, orientationWorldToTerrainFromSurfaceNormalInWorld(normalAndPosition.normal));
    return getProjectedHeadingFrame(eulerXYZ, terrainPlane);
  };

  auto reference2d = generate2DExtrapolatedBaseReference(horizon, initialState, command);

  BaseReferenceTrajectory baseRef;
  baseRef.time = std::move(reference2d.time);
  baseRef.eulerXyz.reserve(horizon.N);
  baseRef.positionInWorld.reserve(horizon.N);

  // Adapt poses
  for (int k = 0; k < horizon.N; ++k) {
    const auto projectedHeadingFrame = getLocalHeadingFrame(reference2d.positionInWorld[k], reference2d.eulerXyz[k]);

    baseRef.positionInWorld.push_back(
        adaptDesiredPositionHeightToTerrain(reference2d.positionInWorld[k], projectedHeadingFrame, command.baseHeight));
    baseRef.eulerXyz.emplace_back(alignDesiredOrientationToTerrain(reference2d.eulerXyz[k], projectedHeadingFrame));
  }

  addVelocitiesFromFiniteDifference(baseRef);
  return baseRef;
}

}  // namespace switched_model