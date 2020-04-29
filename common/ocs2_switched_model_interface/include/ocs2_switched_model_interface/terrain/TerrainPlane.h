//
// Created by rgrandia on 21.04.20.
//

#pragma once

#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

/**
 * Planar terrain model represented by a single coordinate frame.
 * - The terrain frame is located at positionInWorld.
 * - The surface normal points in positive z direction in the terrain frame.
 */
struct TerrainPlane {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  vector3_t positionInWorld = vector3_t::Zero();
  matrix3_t orientationWorldToTerrain = matrix3_t::Identity();

  TerrainPlane() = default;
  TerrainPlane(vector3_t positionInWorld_, matrix3_t orientationWorldToTerrain_)
      : positionInWorld(std::move(positionInWorld_)), orientationWorldToTerrain(std::move(orientationWorldToTerrain_)) {}
};

inline vector3_t surfaceNormalInWorld(const TerrainPlane& terrainPlane) {
  return terrainPlane.orientationWorldToTerrain.row(2).transpose();
}

inline Eigen::Matrix<scalar_t, 2, 3> tangentialBasisFromSurfaceNormal(const vector3_t& surfaceNormal) {
  // Assumes the surface normal is normalized
  Eigen::Matrix<scalar_t, 2, 3> tangentBasis;
  tangentBasis.row(0) = surfaceNormal.cross(vector3_t::Ones()).normalized();  // Cross with any vector that is not equal to surfaceNormal
  tangentBasis.row(1) = surfaceNormal.cross(tangentBasis.row(0));
  return tangentBasis;
}

inline matrix3_t orientationWorldToTerrainFromSurfaceNormalInWorld(const vector3_t& surfaceNormal) {
  Eigen::Matrix<scalar_t, 2, 3> tangents = tangentialBasisFromSurfaceNormal(surfaceNormal);
  matrix3_t orientationWorldToTerrain;
  orientationWorldToTerrain.topRows(2) = tangents;
  orientationWorldToTerrain.row(2) = surfaceNormal.transpose();
  return orientationWorldToTerrain;
}

inline vector3_t positionInTerrainFrameFromPositionInWorld(const vector3_t& positionWorld, const TerrainPlane& terrainPlane) {
  return terrainPlane.orientationWorldToTerrain * (positionWorld - terrainPlane.positionInWorld);
}

inline scalar_t terrainDistanceFromPositionInWorld(const vector3_t& positionWorld, const TerrainPlane& terrainPlane) {
  return positionInTerrainFrameFromPositionInWorld(positionWorld, terrainPlane).z();
}

inline vector3_t projectPositionInWorldOntoPlane(const vector3_t& positionWorld, const TerrainPlane& terrainPlane) {
  const vector3_t surfaceNormal = surfaceNormalInWorld(terrainPlane);
  return surfaceNormal.dot(terrainPlane.positionInWorld - positionWorld) * surfaceNormal + positionWorld;
}

inline vector3_t projectPositionInWorldOntoPlaneAlongGravity(const vector3_t& positionWorld, const TerrainPlane& terrainPlane) {
  const vector3_t surfaceNormal = surfaceNormalInWorld(terrainPlane);
  vector3_t projectedPosition = positionWorld;
  // solve
  // 1. projectedPosition.x() = positionWorld.x();
  // 2. projectedPosition.y() = positionWorld.y();
  // 3. surfaceNormal.dot(projectedPosition - terrainPlane.positionInWorld) = 0
  projectedPosition.z() = (surfaceNormal.dot(terrainPlane.positionInWorld) - projectedPosition.x() * surfaceNormal.x() -
                           projectedPosition.y() * surfaceNormal.y()) /
                          surfaceNormal.z();
  return projectedPosition;
}

TerrainPlane loadTerrainPlane(const std::string& filename, bool verbose);

}  // namespace switched_model
