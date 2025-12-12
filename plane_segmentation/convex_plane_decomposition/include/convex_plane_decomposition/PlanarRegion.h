//
// Created by rgrandia on 10.06.20.
//

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <grid_map_core/GridMap.hpp>

#include "PolygonTypes.h"

namespace convex_plane_decomposition {

struct NormalAndPosition {
  /// 3D position.
  Eigen::Vector3d position;

  /// Surface normal.
  Eigen::Vector3d normal;
};

struct BoundaryWithInset {
  /// Boundary of the planar region.
  CgalPolygonWithHoles2d boundary;

  /// Encodes an inward offset to the boundary.
  std::vector<CgalPolygonWithHoles2d> insets;
};

struct PlanarRegion {
  /// All 2d points are in the terrain frame
  BoundaryWithInset boundaryWithInset;

  /// 2D bounding box in terrain frame containing all the boundary points
  CgalBbox2d bbox2d;

  /// 3D Transformation from terrain to world. v_world = T * v_plane. Use .linear() for the rotation and .translation() for the translation
  Eigen::Isometry3d transformPlaneToWorld;
};

struct PlanarTerrain {
  std::vector<PlanarRegion> planarRegions;
  grid_map::GridMap gridMap;
};

/**
 * Convert a position and normal the a transform from the induced local frame to the global frame.
 *
 * For example, if the normal and position are defined in world. We return a transform T, such that v_world = T * v_plane.
 * The normal will be taken as the z-direction of the local frame. The x and y direction are arbitrary.
 */
Eigen::Isometry3d getTransformLocalToGlobal(const NormalAndPosition& normalAndPosition);

/**
 * Project a 2D point in world along gravity to obtain a 2D point in the plane
 */
CgalPoint2d projectToPlaneAlongGravity(const CgalPoint2d& worldFrameXY, const Eigen::Isometry3d& transformPlaneToWorld);

/**
 * Transforms a point on the plane to a 3D position expressed in the world frame
 */
Eigen::Vector3d positionInWorldFrameFromPosition2dInPlane(const CgalPoint2d& planeXY, const Eigen::Isometry3d& transformPlaneToWorld);

}  // namespace convex_plane_decomposition
