//
// Created by rgrandia on 04.05.20.
//

#pragma once

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/terrain/TerrainPlane.h>

namespace switched_model {

inline vector3_t adaptDesiredPositionHeightToTerrain(const vector3_t& desiredPosition, const TerrainPlane& terrainPlane, scalar_t desiredHeight) {
  const auto adaptedHeight = projectPositionInWorldOntoPlaneAlongGravity(desiredPosition, terrainPlane).z() + desiredHeight;
  return {desiredPosition.x(), desiredPosition.y(), adaptedHeight};
}

inline scalar_t findOrientationClostestToReference(scalar_t yaw, scalar_t reference) {
  while (std::abs(reference - yaw) > M_PI) {
    yaw += std::copysign(scalar_t(2.0 * M_PI), reference - yaw);
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
  return o_R_b.col(0); // x-axis in world frame
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
