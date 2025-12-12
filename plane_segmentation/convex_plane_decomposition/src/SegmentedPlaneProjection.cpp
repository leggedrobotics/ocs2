//
// Created by rgrandia on 12.10.21.
//

#include "convex_plane_decomposition/SegmentedPlaneProjection.h"

#include "convex_plane_decomposition/GeometryUtils.h"

namespace convex_plane_decomposition {

namespace {  // Helper functions that only make sense in this context

double distanceCost(const Eigen::Vector3d& query, const Eigen::Vector3d& terrainPoint) {
  const double dx = query.x() - terrainPoint.x();
  const double dy = query.y() - terrainPoint.y();
  const double dz = query.z() - terrainPoint.z();
  return dx * dx + dy * dy + dz * dz;
}

double distanceCostLowerbound(double distanceSquared) {
  return distanceSquared;
}

double intervalSquareDistance(double value, double min, double max) {
  //   -    |    0     |    +
  //       min        max
  // Returns 0.0 if between min and max. Returns the distance to one boundary otherwise.
  if (value < min) {
    double diff = min - value;
    return diff * diff;
  } else if (value < max) {
    return 0.0;
  } else {
    double diff = max - value;
    return diff * diff;
  }
}

double squaredDistanceToBoundingBox(const CgalPoint2d& point, const CgalBbox2d& boundingBox) {
  const double dxdx = intervalSquareDistance(point.x(), boundingBox.xmin(), boundingBox.xmax());
  const double dydy = intervalSquareDistance(point.y(), boundingBox.ymin(), boundingBox.ymax());
  return dxdx + dydy;
}

const CgalPolygonWithHoles2d* findInsetContainingThePoint(const CgalPoint2d& point, const std::vector<CgalPolygonWithHoles2d>& insets) {
  for (const auto& inset : insets) {
    if (isInside(point, inset.outer_boundary())) {
      return &inset;
    }
  }
  return nullptr;
}

}  // namespace

CgalPoint2d projectToPlanarRegion(const CgalPoint2d& queryProjectedToPlane, const PlanarRegion& planarRegion) {
  // First search if the projected point is inside any of the insets.
  // Note: we know that all insets are non-overlapping, and are not nested (no shape is contained in the hole of another shape)
  const auto* const insetPtrContainingPoint = findInsetContainingThePoint(queryProjectedToPlane, planarRegion.boundaryWithInset.insets);

  // Compute the projection
  CgalPoint2d projectedPoint;
  if (insetPtrContainingPoint == nullptr) {
    // Not inside any of the insets. Need to look for the closest one. The projection will be to the boundary
    double minDistSquared = std::numeric_limits<double>::max();
    for (const auto& inset : planarRegion.boundaryWithInset.insets) {
      const auto closestPoint = projectToClosestPoint(queryProjectedToPlane, inset.outer_boundary());
      double distSquare = squaredDistance(closestPoint, queryProjectedToPlane);
      if (distSquare < minDistSquared) {
        projectedPoint = closestPoint;
        minDistSquared = distSquare;
      }
    }
  } else {
    // Query point is inside and does not need projection...
    projectedPoint = queryProjectedToPlane;

    // ... unless it is inside a hole
    for (const auto& hole : insetPtrContainingPoint->holes()) {
      if (isInside(queryProjectedToPlane, hole)) {
        projectedPoint = projectToClosestPoint(queryProjectedToPlane, hole);
        break;  // No need to search other holes. Holes are not overlapping
      }
    }
  }

  return projectedPoint;
}

std::vector<RegionSortingInfo> sortWithBoundingBoxes(const Eigen::Vector3d& positionInWorld,
                                                     const std::vector<convex_plane_decomposition::PlanarRegion>& planarRegions) {
  // Compute distance to bounding boxes
  std::vector<RegionSortingInfo> regionsAndBboxSquareDistances;
  regionsAndBboxSquareDistances.reserve(planarRegions.size());
  for (const auto& planarRegion : planarRegions) {
    const Eigen::Vector3d positionInTerrainFrame = planarRegion.transformPlaneToWorld.inverse() * positionInWorld;
    const double dzdz = positionInTerrainFrame.z() * positionInTerrainFrame.z();

    RegionSortingInfo regionSortingInfo;
    regionSortingInfo.regionPtr = &planarRegion;
    regionSortingInfo.positionInTerrainFrame = {positionInTerrainFrame.x(), positionInTerrainFrame.y()};
    regionSortingInfo.boundingBoxSquareDistance =
        squaredDistanceToBoundingBox(regionSortingInfo.positionInTerrainFrame, planarRegion.bbox2d) + dzdz;

    regionsAndBboxSquareDistances.push_back(regionSortingInfo);
  }

  // Sort regions close to far
  std::sort(regionsAndBboxSquareDistances.begin(), regionsAndBboxSquareDistances.end(),
            [](const RegionSortingInfo& lhs, const RegionSortingInfo& rhs) {
              return lhs.boundingBoxSquareDistance < rhs.boundingBoxSquareDistance;
            });

  return regionsAndBboxSquareDistances;
}

PlanarTerrainProjection getBestPlanarRegionAtPositionInWorld(const Eigen::Vector3d& positionInWorld,
                                                             const std::vector<PlanarRegion>& planarRegions,
                                                             const std::function<double(const Eigen::Vector3d&)>& penaltyFunction) {
  const auto sortedRegions = sortWithBoundingBoxes(positionInWorld, planarRegions);

  // Look for closest planar region.
  PlanarTerrainProjection projection;
  projection.cost = std::numeric_limits<double>::max();
  for (const auto& regionInfo : sortedRegions) {
    // Skip based on lower bound
    if (distanceCostLowerbound(regionInfo.boundingBoxSquareDistance) > projection.cost) {
      continue;
    }

    // Project onto planar region
    const auto projectedPointInTerrainFrame = projectToPlanarRegion(regionInfo.positionInTerrainFrame, *regionInfo.regionPtr);

    // Express projected point in World frame
    const auto projectionInWorldFrame =
        positionInWorldFrameFromPosition2dInPlane(projectedPointInTerrainFrame, regionInfo.regionPtr->transformPlaneToWorld);

    const auto cost = distanceCost(positionInWorld, projectionInWorldFrame) + penaltyFunction(projectionInWorldFrame);
    if (cost < projection.cost) {
      projection.cost = cost;
      projection.regionPtr = regionInfo.regionPtr;
      projection.positionInTerrainFrame = projectedPointInTerrainFrame;
      projection.positionInWorld = projectionInWorldFrame;
    }
  }

  return projection;
}

}  // namespace convex_plane_decomposition