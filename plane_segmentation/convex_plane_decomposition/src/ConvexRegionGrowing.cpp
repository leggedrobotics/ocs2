//
// Created by rgrandia on 09.06.20.
//

#include "convex_plane_decomposition/ConvexRegionGrowing.h"

#include "convex_plane_decomposition/GeometryUtils.h"

namespace convex_plane_decomposition {

namespace {

int getCounterClockWiseNeighbour(int i, int lastVertex) {
  return (i > 0) ? i - 1 : lastVertex;
}

int getClockWiseNeighbour(int i, int lastVertex) {
  return (i < lastVertex) ? i + 1 : 0;
}

std::array<CgalPoint2d, 2> getNeighbours(const CgalPolygon2d& polygon, int i) {
  assert(i < polygon.size());
  assert(polygon.size() > 1);
  int lastVertex = static_cast<int>(polygon.size()) - 1;
  int cwNeighbour = getClockWiseNeighbour(i, lastVertex);
  int ccwNeighbour = getCounterClockWiseNeighbour(i, lastVertex);
  return {polygon.vertex(cwNeighbour), polygon.vertex(ccwNeighbour)};
}

std::array<CgalPoint2d, 4> get2ndNeighbours(const CgalPolygon2d& polygon, int i) {
  assert(i < polygon.size());
  assert(polygon.size() > 1);
  int lastVertex = static_cast<int>(polygon.size()) - 1;
  int cwNeighbour1 = getClockWiseNeighbour(i, lastVertex);
  int cwNeighbour2 = getClockWiseNeighbour(cwNeighbour1, lastVertex);
  int ccwNeighbour1 = getCounterClockWiseNeighbour(i, lastVertex);
  int ccwNeighbour2 = getCounterClockWiseNeighbour(ccwNeighbour1, lastVertex);
  return {polygon.vertex(cwNeighbour2), polygon.vertex(cwNeighbour1), polygon.vertex(ccwNeighbour1), polygon.vertex(ccwNeighbour2)};
}

bool remainsConvexWhenMovingPoint(const CgalPolygon2d& polygon, int i, const CgalPoint2d& point) {
  auto secondNeighbours = get2ndNeighbours(polygon, i);
  using CgalPolygon2dFixedSize = CGAL::Polygon_2<K, std::array<K::Point_2, 5>>;

  CgalPolygon2dFixedSize subPolygon;
  subPolygon.container()[0] = secondNeighbours[0];
  subPolygon.container()[1] = secondNeighbours[1];
  subPolygon.container()[2] = point;
  subPolygon.container()[3] = secondNeighbours[2];
  subPolygon.container()[4] = secondNeighbours[3];
  return subPolygon.is_convex();
}

bool pointAndNeighboursAreWithinFreeSphere(const std::array<CgalPoint2d, 2>& neighbours, const CgalPoint2d& point,
                                           const CgalCircle2d& circle) {
  return isInside(neighbours[0], circle) && isInside(point, circle) && isInside(neighbours[1], circle);
}

/**
 * Returns {true, 0.0} if either one of the point -> neighbour segments intersects the parent shape
 * Returns {false, minSquareDistance} if none of the segments intersects. minSquareDistance is the minimum square distance between the
 * point and the parent shape
 */
template <typename T>
std::pair<bool, double> doEdgesIntersectAndSquareDistance(const std::array<CgalPoint2d, 2>& neighbours, const CgalPoint2d& point,
                                                          const T& parentShape);

template <>
std::pair<bool, double> doEdgesIntersectAndSquareDistance(const std::array<CgalPoint2d, 2>& neighbours, const CgalPoint2d& point,
                                                          const CgalPolygon2d& parentShape) {
  CgalSegment2d segment0{neighbours[0], point};
  CgalSegment2d segment1{neighbours[1], point};

  double minDistSquared = std::numeric_limits<double>::max();
  for (auto edgeIt = parentShape.edges_begin(); edgeIt != parentShape.edges_end(); ++edgeIt) {
    const auto edge = *edgeIt;
    if (CGAL::do_intersect(segment0, edge) || CGAL::do_intersect(segment1, edge)) {
      return {true, 0.0};
    } else {
      minDistSquared = std::min(minDistSquared, CGAL::squared_distance(point, edge));
    }
  }

  return {false, minDistSquared};
}

template <>
std::pair<bool, double> doEdgesIntersectAndSquareDistance(const std::array<CgalPoint2d, 2>& neighbours, const CgalPoint2d& point,
                                                          const CgalPolygonWithHoles2d& parentShape) {
  const auto intersectAndDistance = doEdgesIntersectAndSquareDistance(neighbours, point, parentShape.outer_boundary());
  if (intersectAndDistance.first) {
    return {true, 0.0};
  }

  double minDistSquared = intersectAndDistance.second;
  for (const auto& hole : parentShape.holes()) {
    const auto holeIntersectAndDistance = doEdgesIntersectAndSquareDistance(neighbours, point, hole);
    if (holeIntersectAndDistance.first) {
      return {true, 0.0};
    } else {
      minDistSquared = std::min(minDistSquared, holeIntersectAndDistance.second);
    }
  }

  return {false, minDistSquared};
}

template <typename T>
bool pointCanBeMoved(const CgalPolygon2d& growthShape, int i, const CgalPoint2d& candidatePoint, CgalCircle2d& freeSphere,
                     const T& parentShape) {
  if (remainsConvexWhenMovingPoint(growthShape, i, candidatePoint)) {
    auto neighbours = getNeighbours(growthShape, i);
    if (pointAndNeighboursAreWithinFreeSphere(neighbours, candidatePoint, freeSphere)) {
      return true;
    } else {
      // Look for intersections and minimum distances simultaneously
      const auto intersectAndDistance = doEdgesIntersectAndSquareDistance(neighbours, candidatePoint, parentShape);

      if (intersectAndDistance.first) {
        return false;
      } else {
        // Update free sphere around new point
        freeSphere = CgalCircle2d(candidatePoint, intersectAndDistance.second);
        return true;
      }
    }
  } else {
    return false;
  }
}

inline std::ostream& operator<<(std::ostream& os, const CgalPolygon2d& p) {
  os << "CgalPolygon2d: \n";
  for (auto it = p.vertices_begin(); it != p.vertices_end(); ++it) {
    os << "\t(" << it->x() << ", " << it->y() << ")\n";
  }
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const CgalPolygonWithHoles2d& p) {
  os << "CgalPolygonWithHoles2d: \n";
  os << "\t" << p.outer_boundary() << "\n";
  os << "\tHoles: \n";
  for (auto it = p.holes_begin(); it != p.holes_end(); ++it) {
    os << "\t\t" << *it << "\n";
  }
  return os;
}

template <typename T>
CgalPolygon2d growConvexPolygonInsideShape_impl(const T& parentShape, CgalPoint2d center, int numberOfVertices, double growthFactor) {
  const auto centerCopy = center;
  constexpr double initialRadiusFactor = 0.999;
  constexpr int maxIter = 1000;
  double radius = initialRadiusFactor * distance(center, parentShape);

  CgalPolygon2d growthShape = createRegularPolygon(center, radius, numberOfVertices);

  if (radius == 0.0) {
    std::cerr << "[growConvexPolygonInsideShape] Zero initial radius. Provide a point with a non-zero offset to the boundary.\n";
    return growthShape;
  }

  // Cached values per vertex
  std::vector<bool> blocked(numberOfVertices, false);
  std::vector<CgalCircle2d> freeSpheres(numberOfVertices, CgalCircle2d(center, radius * radius));

  int Nblocked = 0;
  int iter = 0;
  while (Nblocked < numberOfVertices && iter < maxIter) {
    for (int i = 0; i < numberOfVertices; i++) {
      if (!blocked[i]) {
        const auto candidatePoint = getPointOnLine(center, growthShape.vertex(i), growthFactor);
        if (pointCanBeMoved(growthShape, i, candidatePoint, freeSpheres[i], parentShape)) {
          updateMean(center, growthShape.vertex(i), candidatePoint, numberOfVertices);
          growthShape.vertex(i) = candidatePoint;
        } else {
          blocked[i] = true;
          ++Nblocked;
        }
      }
    }
    ++iter;
  }

  if (iter == maxIter) {
    std::cerr << "[growConvexPolygonInsideShape] max iteration in region growing! Debug information: \n";
    std::cerr << "numberOfVertices: " << numberOfVertices << "\n";
    std::cerr << "growthFactor: " << growthFactor << "\n";
    std::cerr << "Center: " << centerCopy.x() << ", " << centerCopy.y() << "\n";
    std::cerr << parentShape << "\n";
  }
  return growthShape;
}

}  // namespace

CgalPolygon2d createRegularPolygon(const CgalPoint2d& center, double radius, int numberOfVertices) {
  assert(numberOfVertices > 2);
  CgalPolygon2d polygon;
  polygon.container().reserve(numberOfVertices);
  double angle = (2. * M_PI) / numberOfVertices;
  for (int i = 0; i < numberOfVertices; ++i) {
    double phi = i * angle;
    double px = radius * std::cos(phi) + center.x();
    double py = radius * std::sin(phi) + center.y();
    // Counter clockwise
    polygon.push_back({px, py});
  }
  return polygon;
}

CgalPolygon2d growConvexPolygonInsideShape(const CgalPolygon2d& parentShape, CgalPoint2d center, int numberOfVertices,
                                           double growthFactor) {
  return growConvexPolygonInsideShape_impl(parentShape, center, numberOfVertices, growthFactor);
}

CgalPolygon2d growConvexPolygonInsideShape(const CgalPolygonWithHoles2d& parentShape, CgalPoint2d center, int numberOfVertices,
                                           double growthFactor) {
  return growConvexPolygonInsideShape_impl(parentShape, center, numberOfVertices, growthFactor);
}

void updateMean(CgalPoint2d& mean, const CgalPoint2d& oldValue, const CgalPoint2d& updatedValue, int N) {
  // old_mean = 1/N * ( others + old_value); -> others = N*old_mean - old_value
  // new_mean = 1/N * ( others + new_value); -> new_mean = old_mean - 1/N * oldValue + 1/N * updatedValue
  mean += 1.0 / N * (updatedValue - oldValue);
}

}  // namespace convex_plane_decomposition
