//
// Created by rgrandia on 04.05.20.
//

#pragma once

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/terrain/TerrainPlane.h>

namespace switched_model {

/**
 * Creates a desired position at a height above the given terrain. The x-y coordinated remain the same
 */
vector3_t adaptDesiredPositionHeightToTerrain(const vector3_t& desiredPosition, const TerrainPlane& terrainPlane, scalar_t desiredHeight);

/**
 * Creates a desired position at a height above the given terrain. The x-y coordinated remain the same
 */
vector3_t adaptDesiredPositionHeightToTerrain(const vector2_t& desiredXYPosition, const TerrainPlane& terrainPlane, scalar_t desiredHeight);

/**
 * Return euler angles XYZ from a rotation matrix. When a reference yaw is given, the yaw angle is chosen as close as possible to the
 * reference.
 */
vector3_t eulerXYZFromRotationMatrix(const matrix3_t& orientationTargetToWorld, scalar_t referenceYaw = 0.0);

/**
 * Return the x-axis of the body frame expressed in world coordinates.
 */
vector3_t getHeadingVectorInWorld(const vector3_t& eulerXYZ);

/**
 * Compute the yaw angle around the world Z that leads to the current heading projected to the XY plane
 *
 * @param eulerXYZ : current body eulerXYZ from which the heading vector is derived.
 * @return Angle between the world X axis and the projected heading frame
 */
scalar_t getHeadingAngleInWorld(const vector3_t& eulerXYZ);

/**
 * Returns a rotation matrix from the "projected heading frame" to world. See definition below.
 */
matrix3_t getOrientationProjectedHeadingFrameToWorld(const vector3_t& headingVector, const TerrainPlane& terrainPlane);

/**
 * The heading of the robot refers to the x-axis of the body frame.
 *
 * The "projected heading frame" is defined as follows:
 * - the x-Axis points in same direction as the given x-axis of the body frame in world when both are projected to the world XY plane.
 * - z-Axis is the surface normal.
 * - y-Axis is found from the right hand rule.
 * - the origin is the same as to origin of the terrain projected to.
 *
 * @param eulerXYZ : current body eulerXYZ from which the heading vector is derived.
 * @param terrainPlane : plane to project the heading on.
 * @return projected heading frame
 */
TerrainPlane getProjectedHeadingFrame(const vector3_t& eulerXYZ, const TerrainPlane& terrainPlane);

/**
 * Modifies a desired orientation such that the body z-axis is aligned with the surface normal and the heading vector (x-axis) projected
 * onto the terrain still points in the same direction. The orientation is the same as that of the projected heading frame defined above.
 * @return adapted euler angles XYZ
 */
vector3_t alignDesiredOrientationToTerrain(const vector3_t& desiredEulerXYZ, const TerrainPlane& terrainPlane);

/**
 * Advances an orientation w.r.t world frame by applying a rotation around a given axis
 * @return integrated eulerXYZ (continuous yaw) target to world
 */
vector3_t advanceOrientationInWorld(const vector3_t& eulerXYZ, const vector3_t& unitRotationAxisInWorld, double angle);

}  // namespace switched_model
