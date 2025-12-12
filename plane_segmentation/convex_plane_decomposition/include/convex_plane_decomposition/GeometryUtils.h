//
// Created by rgrandia on 09.06.20.
//

#pragma once

#include "PlanarRegion.h"
#include "PolygonTypes.h"

namespace convex_plane_decomposition {

inline bool doEdgesIntersect(const CgalSegment2d& line, const CgalPolygon2d& contour) {
  for (auto edgeIt = contour.edges_begin(); edgeIt != contour.edges_end(); ++edgeIt) {
    if (CGAL::do_intersect(line, *edgeIt)) {
      return true;
    }
  }
  return false;
}

inline bool doEdgesIntersect(const CgalSegment2d& line, const CgalPolygonWithHoles2d& parentShape) {
  if (doEdgesIntersect(line, parentShape.outer_boundary())) {
    return true;
  } else {
    for (const auto& hole : parentShape.holes()) {
      if (doEdgesIntersect(line, hole)) {
        return true;
      }
    }
  }
  return false;
}

inline double squaredDistance(const CgalPoint2d& point, const CgalPolygon2d& polygon) {
  double minDistSquared = std::numeric_limits<double>::max();
  for (auto edgeIt = polygon.edges_begin(); edgeIt != polygon.edges_end(); ++edgeIt) {
    double distSquare = CGAL::squared_distance(point, *edgeIt);
    minDistSquared = std::min(distSquare, minDistSquared);
  }
  return minDistSquared;
}

inline double squaredDistance(const CgalPoint2d& point, const CgalPolygonWithHoles2d& parentShape) {
  double minDistSquared = squaredDistance(point, parentShape.outer_boundary());
  for (const auto& hole : parentShape.holes()) {
    double distSquare = squaredDistance(point, hole);
    minDistSquared = std::min(distSquare, minDistSquared);
  }
  return minDistSquared;
}

inline double squaredDistance(const CgalPoint2d& point, const CgalCircle2d& circle) {
  auto dx = (point.x() - circle.center().x());
  auto dy = (point.y() - circle.center().y());
  return dx * dx + dy * dy;
}

template <typename T>
double distance(const CgalPoint2d& point, const T& shape) {
  double distanceSquared = squaredDistance(point, shape);
  return (distanceSquared > 0.0) ? std::sqrt(distanceSquared) : 0.0;
}

inline bool isInside(const CgalPoint2d& point, const CgalCircle2d& circle) {
  return squaredDistance(point, circle) <= circle.squared_radius();
}

inline bool isInside(const CgalPoint2d& point, const CgalPolygon2d& polygon) {
  const auto boundedSide = CGAL::bounded_side_2(polygon.begin(), polygon.end(), point);
  return boundedSide == CGAL::ON_BOUNDED_SIDE || boundedSide == CGAL::ON_BOUNDARY;
}

inline bool isInside(const CgalPoint2d& point, const CgalPolygonWithHoles2d& polygonWithHoles) {
  if (isInside(point, polygonWithHoles.outer_boundary())) {
    // Inside the outer contour -> return false if the point is inside any of the holes
    for (const auto& hole : polygonWithHoles.holes()) {
      const auto boundedSide = CGAL::bounded_side_2(hole.begin(), hole.end(), point);
      if (boundedSide == CGAL::ON_BOUNDED_SIDE) {  // The edge of the hole is considered part of the polygon
        return false;
      }
    }
    return true;
  } else {
    return false;
  }
}

inline CgalPoint2d getPointOnLine(const CgalPoint2d& start, const CgalPoint2d& end, double factor) {
  return {factor * (end.x() - start.x()) + start.x(), factor * (end.y() - start.y()) + start.y()};
}

inline CgalPoint2d projectToClosestPoint(const CgalPoint2d& point, const CgalSegment2d& segment) {
  // The segment as a vector, with the source being the origin
  const Eigen::Vector2d sourceToTarget{segment.target().x() - segment.source().x(), segment.target().y() - segment.source().y()};
  const double sourceToTargetDistance = sourceToTarget.norm();
  const Eigen::Vector2d n = sourceToTarget / sourceToTargetDistance;

  // Vector from source to the query point
  const Eigen::Vector2d sourceToPoint{point.x() - segment.source().x(), point.y() - segment.source().y()};

  // Projection to the line, clamped to be between source and target points
  const double coeff = std::min(std::max(0.0, n.dot(sourceToPoint)), sourceToTargetDistance);

  return {coeff * n.x() + segment.source().x(), coeff * n.y() + segment.source().y()};
}

inline CgalPoint2d projectToClosestPoint(const CgalPoint2d& point, const CgalPolygon2d& polygon) {
  double minDistSquared = CGAL::squared_distance(point, *polygon.edges_begin());
  auto closestEdge = polygon.edges_begin();
  for (auto edgeIt = std::next(polygon.edges_begin()); edgeIt != polygon.edges_end(); ++edgeIt) {
    double distSquare = CGAL::squared_distance(point, *edgeIt);
    if (distSquare < minDistSquared) {
      closestEdge = edgeIt;
      minDistSquared = distSquare;
    }
  }
  return projectToClosestPoint(point, *closestEdge);
}

inline void transformInPlace(CgalPolygon2d& polygon, const std::function<void(CgalPoint2d&)>& f) {
  for (auto& point : polygon) {
    f(point);
  }
}

inline void transformInPlace(CgalPolygonWithHoles2d& polygonWithHoles, const std::function<void(CgalPoint2d&)>& f) {
  transformInPlace(polygonWithHoles.outer_boundary(), f);
  for (auto& hole : polygonWithHoles.holes()) {
    transformInPlace(hole, f);
  }
}

inline void transformInPlace(BoundaryWithInset& boundaryWithInset, const std::function<void(CgalPoint2d&)>& f) {
  transformInPlace(boundaryWithInset.boundary, f);
  for (auto& inset : boundaryWithInset.insets) {
    transformInPlace(inset, f);
  }
}

}  // namespace convex_plane_decomposition
