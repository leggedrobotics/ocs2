//
// Created by rgrandia on 14.05.20.
//

#include "ocs2_anymal_commands/TerrainAdaptation.h"

namespace switched_model {

matrix3_t getOrientationProjectedHeadingFrameToWorld(const vector3_t& headingVector, const TerrainPlane& terrainPlane) {
  // Construct desired axis system
  // x-Axis points in same direction as the original x axis in world when both are projected to the world XY plane
  // z-Axis is the surface normal
  const vector3_t xAxisProjectedHeadingFrame = projectVectorInWorldOntoPlaneAlongGravity(headingVector, terrainPlane).normalized();
  const vector3_t zAxisProjectedHeadingFrame = surfaceNormalInWorld(terrainPlane);
  const vector3_t yAxisProjectedHeadingFrame = zAxisProjectedHeadingFrame.cross(xAxisProjectedHeadingFrame);

  // Construct rotation matrix from desired axis system
  matrix3_t o_R_projectedheading;
  o_R_projectedheading.col(0) = xAxisProjectedHeadingFrame;
  o_R_projectedheading.col(1) = yAxisProjectedHeadingFrame;
  o_R_projectedheading.col(2) = zAxisProjectedHeadingFrame;
  return o_R_projectedheading;
}

vector3_t alignDesiredOrientationToTerrain(const vector3_t& desiredEulerXYZ, const TerrainPlane& terrainPlane) {
  // Construct rotation matrix from desired orientation projected to terrain
  const vector3_t xAxisInWorld = getHeadingVectorInWorld(desiredEulerXYZ);
  matrix3_t o_R_adapted = getOrientationProjectedHeadingFrameToWorld(xAxisInWorld, terrainPlane);

  // Convert rotation matrix back to euler angles with the yaw close to the desired one
  return eulerXYZFromRotationMatrix(o_R_adapted, desiredEulerXYZ.z());
}

vector3_t advanceOrientationInWorld(const vector3_t& eulerXYZ, const vector3_t& unitRotationAxisInWorld, double angle) {
  const matrix3_t world_R_curr = rotationMatrixBaseToOrigin(eulerXYZ);

  // Get rotation around the surface normal in the current frame
  const matrix3_t delta_R = Eigen::AngleAxis<scalar_t>(angle, unitRotationAxisInWorld).toRotationMatrix();

  // Concatenate rotations
  const matrix3_t world_R_new = delta_R * world_R_curr;
  const vector3_t advancedEulerXYZ = eulerXYZFromRotationMatrix(world_R_new, eulerXYZ.z());

  return advancedEulerXYZ;
}

}  // namespace switched_model
