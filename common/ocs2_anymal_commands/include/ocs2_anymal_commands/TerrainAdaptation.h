//
// Created by rgrandia on 04.05.20.
//

#pragma once

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/terrain/TerrainPlane.h>

namespace switched_model {

inline vector3_t adaptDesiredPositionHeightToTerrain(const vector3_t& desiredPosition, const TerrainPlane& terrainPlane, const scalar_t desiredHeight) {
  // Project to plane along gravity, interpret old z as desired height offset.
  vector3_t adaptedPosition = projectPositionInWorldOntoPlaneAlongGravity(desiredPosition, terrainPlane);
  adaptedPosition.z() += desiredHeight;
  return adaptedPosition;
}

inline scalar_t findOrientationClostestToReference(scalar_t yaw, scalar_t reference) {
  while (std::abs(yaw - reference) > M_PI) {
    yaw -= (yaw - reference) / std::abs(yaw - reference) * 2 * M_PI;
  }
  return yaw;
}

inline vector3_t eulerXYZFromRotationMatrix(const matrix3_t& orientationTargetToWorld, scalar_t referenceYaw = 0.0) {
  vector3_t eulerXYZ = orientationTargetToWorld.eulerAngles(0, 1, 2);
  ocs2::makeEulerAnglesUnique(eulerXYZ);
  eulerXYZ.z() = findOrientationClostestToReference(eulerXYZ.z(), referenceYaw);
  return eulerXYZ;
}

inline vector3_t getHeadingVectorInWorld(const vector3_t& eulerXYZ) {
  const matrix3_t o_R_b = rotationMatrixBaseToOrigin(eulerXYZ);
  const vector3_t xAxisInWorld = o_R_b.col(0);
  return xAxisInWorld;
}

matrix3_t getOrientationProjectedHeadingFrameToWorld(const vector3_t& headingVector, const TerrainPlane& terrainPlane);

inline TerrainPlane getProjectedHeadingFrame(const vector3_t& eulerXYZ, const TerrainPlane& terrainPlane) {
  const vector3_t xAxisInWorld = getHeadingVectorInWorld(eulerXYZ);
  return {terrainPlane.positionInWorld, getOrientationProjectedHeadingFrameToWorld(xAxisInWorld, terrainPlane).transpose()};
}

vector3_t alignDesiredOrientationToTerrain(const vector3_t& desiredEulerXYZ, const TerrainPlane& terrainPlane);

/**
 * Advances an orientation w.r.t world frame by applying a rotation around a given axis
 * @return integrated eulerXYZ (continuous yaw) target to world
 */
vector3_t advanceOrientationInWorld(const vector3_t& eulerXYZ, const vector3_t& unitRotationAxisInWorld, double angle);

}  // namespace switched_model
