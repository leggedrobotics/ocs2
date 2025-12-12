#include "convex_plane_decomposition/PlanarRegion.h"

namespace convex_plane_decomposition {

Eigen::Isometry3d getTransformLocalToGlobal(const NormalAndPosition& normalAndPosition) {
  const Eigen::Vector3d zAxisInGlobal = normalAndPosition.normal.normalized();
  const auto& positionInGlobal = normalAndPosition.position;

  // Cross with any vector that is not equal to surfaceNormal
  Eigen::Vector3d yAxisInGlobal = zAxisInGlobal.cross(Eigen::Vector3d::UnitX());
  {  // Normalize the yAxis. Need to pick a different direction if z happened to intersect with unitX
    const auto ySquaredNorm = yAxisInGlobal.squaredNorm();
    const double crossTolerance = 1e-3;
    if (ySquaredNorm > crossTolerance) {
      yAxisInGlobal /= std::sqrt(ySquaredNorm);
    } else {
      // normal was almost equal to unitX. Pick the y-axis in a different way (approximately equal to unitY):
      yAxisInGlobal = zAxisInGlobal.cross(Eigen::Vector3d::UnitY().cross(zAxisInGlobal)).normalized();
    }
  }

  Eigen::Isometry3d transform;
  transform.linear().col(0) = yAxisInGlobal.cross(zAxisInGlobal);
  transform.linear().col(1) = yAxisInGlobal;
  transform.linear().col(2) = zAxisInGlobal;
  transform.translation() = positionInGlobal;

  return transform;
}

CgalPoint2d projectToPlaneAlongGravity(const CgalPoint2d& worldFrameXY, const Eigen::Isometry3d& transformPlaneToWorld) {
  // Shorthands
  const auto& xAxis = transformPlaneToWorld.linear().col(0);
  const auto& yAxis = transformPlaneToWorld.linear().col(1);
  const auto& surfaceNormalInWorld = transformPlaneToWorld.linear().col(2);
  const auto& planeOriginInWorld = transformPlaneToWorld.translation();

  // Horizontal difference
  const double dx = worldFrameXY.x() - planeOriginInWorld.x();
  const double dy = worldFrameXY.y() - planeOriginInWorld.y();

  // Vertical difference
  // solve surfaceNormalInWorld.dot(projectedPosition - planeOriginInWorld) = 0
  // with projectPosition XY = worldFrameXY;
  Eigen::Vector3d planeOriginToProjectedPointInWorld(
      dx, dy, (-dx * surfaceNormalInWorld.x() - dy * surfaceNormalInWorld.y()) / surfaceNormalInWorld.z());

  // Project XY coordinates to the plane frame
  return {xAxis.dot(planeOriginToProjectedPointInWorld), yAxis.dot(planeOriginToProjectedPointInWorld)};
}

Eigen::Vector3d positionInWorldFrameFromPosition2dInPlane(const CgalPoint2d& planeXY, const Eigen::Isometry3d& transformPlaneToWorld) {
  // Compute transform given that z in plane = 0.0
  Eigen::Vector3d pointInWorld = transformPlaneToWorld.translation() + planeXY.x() * transformPlaneToWorld.linear().col(0) +
                                 planeXY.y() * transformPlaneToWorld.linear().col(1);
  return pointInWorld;
}

}  // namespace convex_plane_decomposition