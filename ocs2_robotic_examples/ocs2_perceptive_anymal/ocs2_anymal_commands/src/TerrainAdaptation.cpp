//
// Created by rgrandia on 14.05.20.
//

#include "ocs2_anymal_commands/TerrainAdaptation.h"

namespace switched_model {

vector3_t adaptDesiredPositionHeightToTerrain(const vector3_t& desiredPosition, const TerrainPlane& terrainPlane, scalar_t desiredHeight) {
  return adaptDesiredPositionHeightToTerrain(vector2_t{desiredPosition.x(), desiredPosition.y()}, terrainPlane, desiredHeight);
}

vector3_t adaptDesiredPositionHeightToTerrain(const vector2_t& desiredXYPosition, const TerrainPlane& terrainPlane,
                                              scalar_t desiredHeight) {
  const auto adaptedHeight = projectPositionInWorldOntoPlaneAlongGravity(desiredXYPosition, terrainPlane).z() + desiredHeight;
  return {desiredXYPosition.x(), desiredXYPosition.y(), adaptedHeight};
}

vector3_t eulerXYZFromRotationMatrix(const matrix3_t& orientationTargetToWorld, scalar_t referenceYaw) {
  vector3_t eulerXYZ = orientationTargetToWorld.eulerAngles(0, 1, 2);
  ocs2::makeEulerAnglesUnique(eulerXYZ);
  eulerXYZ.z() = ocs2::moduloAngleWithReference(eulerXYZ.z(), referenceYaw);
  return eulerXYZ;
}

vector3_t getHeadingVectorInWorld(const vector3_t& eulerXYZ) {
  return rotationMatrixBaseToOrigin(eulerXYZ).col(0);  // x-axis in world frame
}

scalar_t getHeadingAngleInWorld(const vector3_t& eulerXYZ) {
  // Align to a horizontal terrain. Return the yaw
  return alignDesiredOrientationToTerrain(eulerXYZ, TerrainPlane()).z();
}

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

TerrainPlane getProjectedHeadingFrame(const vector3_t& eulerXYZ, const TerrainPlane& terrainPlane) {
  const vector3_t xAxisInWorld = getHeadingVectorInWorld(eulerXYZ);
  return {terrainPlane.positionInWorld, getOrientationProjectedHeadingFrameToWorld(xAxisInWorld, terrainPlane).transpose()};
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
