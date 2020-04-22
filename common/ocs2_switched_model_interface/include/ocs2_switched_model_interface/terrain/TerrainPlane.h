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
  vector3_t positionInWorld = vector3_t::Zero();
  matrix3_t orientationWorldToTerrain = matrix3_t::Identity();
};

inline vector3_t surfaceNormalInWorld(const TerrainPlane& terrainPlane) {
  return terrainPlane.orientationWorldToTerrain.row(2).transpose();
}

inline vector3_t positionInTerrainFrameFromPositionInWorld(const vector3_t& positionWorld, const TerrainPlane& terrainPlane) {
  return terrainPlane.orientationWorldToTerrain * (positionWorld - terrainPlane.positionInWorld);
}

inline scalar_t terrainDistanceFromPositionInWorld(const vector3_t& positionWorld, const TerrainPlane& terrainPlane) {
  return positionInTerrainFrameFromPositionInWorld(positionWorld, terrainPlane).z();
}

TerrainPlane loadTerrainPlane(const std::string& filename, bool verbose);

}  // namespace switched_model
