//
// Created by rgrandia on 12.10.21.
//

#pragma once

#include <functional>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition/PolygonTypes.h>

namespace convex_plane_decomposition {

/**
 * Projects a point in the plane to the closest point on the contour of a planar region. We take the inset (slight inward offset from the
 * boundary) as the contour to project to.
 * @param queryProjectedToPlane : 2D point in the frame of the planar regions
 * @param planarRegion : planar region to project to
 * @return projected 2D point in the frame of the planar regions
 */
CgalPoint2d projectToPlanarRegion(const CgalPoint2d& queryProjectedToPlane, const PlanarRegion& planarRegion);

/**
 * Sorting information as an intermediate step to find the best plane projection
 */
struct RegionSortingInfo {
  const PlanarRegion* regionPtr{nullptr};
  CgalPoint2d positionInTerrainFrame{0.0, 0.0};
  double boundingBoxSquareDistance{0.0};
};

/**
 * Compute sorting info and sort according to the bounding box distances.
 *
 * @param positionInWorld : Query point in world frame
 * @param planarRegions : Candidate planar regions
 * @return RegionSortingInfo, sorted according to the bounding box distance
 */
std::vector<RegionSortingInfo> sortWithBoundingBoxes(const Eigen::Vector3d& positionInWorld,
                                                     const std::vector<PlanarRegion>& planarRegions);

struct PlanarTerrainProjection {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Selected region
  const PlanarRegion* regionPtr{nullptr};

  /// Projected point in terrain frame of the selected region
  CgalPoint2d positionInTerrainFrame{0.0, 0.0};

  /// Projected point in world frame
  Eigen::Vector3d positionInWorld{0.0, 0.0, 0.0};

  /// Projection cost, see getBestPlanarRegionAtPositionInWorld
  double cost{0.0};
};

/**
 * This function considers the projection of a 3D query point to a set of candidate regions.
 *
 * The "best" region is picked according to the following cost:
 *      cost = |p - p_projected|^2 + penaltyFunction(p_projected),
 *  where p is the query point, and p_projected is the Euclidean projection to the candidate region, both in world frame.
 *
 *  The bounding box of each region is used to find a lower bound on this cost, it is therefore important the user defined penalty is
 * non-negative.
 *
 * @param positionInWorld : Query point in world frame
 * @param planarRegions : Candidate planar regions
 * @param penaltyFunction : a non-negative (!) scoring function.
 * @return Projection and information
 */
PlanarTerrainProjection getBestPlanarRegionAtPositionInWorld(const Eigen::Vector3d& positionInWorld,
                                                             const std::vector<PlanarRegion>& planarRegions,
                                                             const std::function<double(const Eigen::Vector3d&)>& penaltyFunction);

}  // namespace convex_plane_decomposition