//
// Created by rgrandia on 04.05.20.
//

#pragma once

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/terrain/TerrainPlane.h>

namespace switched_model {

inline vector3_t adaptDesiredPositionToTerrain(const vector3_t& desiredPosition, const TerrainPlane& terrainPlane) {
  // Project to plane along gravity, interpret old z as desired height offset.
  vector3_t adaptedPosition = projectPositionInWorldOntoPlaneAlongGravity(desiredPosition, terrainPlane);
  adaptedPosition.z() += desiredPosition.z();
  return adaptedPosition;
}

inline scalar_t findOrientationClostestToReference(scalar_t yaw, scalar_t reference) {
  while (std::abs(yaw - reference) > M_PI) {
    yaw -= (yaw - reference) / std::abs(yaw - reference) * 2 * M_PI;
  }
  return yaw;
}

inline vector3_t adaptDesiredOrientationToTerrain(const vector3_t& desiredEulerXYZ, const TerrainPlane& terrainPlane) {
  const matrix3_t o_R_b = rotationMatrixBaseToOrigin(desiredEulerXYZ);
  const vector3_t xAxisInWorldDesiredOrientation = o_R_b.col(0);

  TerrainPlane terrainOrientationOnly{vector3_t::Zero(), terrainPlane.orientationWorldToTerrain};

  // Construct desired axis system
  // x-Axis points in same direction as desired x axis
  // z-Axis is the surface normal
  const vector3_t xAxisAdaptedOrientation =
      projectPositionInWorldOntoPlaneAlongGravity(xAxisInWorldDesiredOrientation, terrainOrientationOnly).normalized();
  const vector3_t zAxisAdaptedOrientation = surfaceNormalInWorld(terrainOrientationOnly);
  const vector3_t yAxisAdaptedOrientation = zAxisAdaptedOrientation.cross(xAxisAdaptedOrientation);

  // Construct rotation matrix from desired axis system
  matrix3_t o_R_adapted;
  o_R_adapted.col(0) = xAxisAdaptedOrientation;
  o_R_adapted.col(1) = yAxisAdaptedOrientation;
  o_R_adapted.col(2) = zAxisAdaptedOrientation;

  // Convert back to euler angles
  vector3_t adaptedEulerXYZ = o_R_adapted.eulerAngles(0, 1, 2);
  ocs2::makeEulerAnglesUnique(adaptedEulerXYZ);
  adaptedEulerXYZ.z() = findOrientationClostestToReference(adaptedEulerXYZ.z(), desiredEulerXYZ.z());
  return adaptedEulerXYZ;
}

}  // namespace switched_model
