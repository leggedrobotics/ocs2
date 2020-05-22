//
// Created by rgrandia on 29.04.20.
//

#include "ocs2_switched_model_interface/terrain/TerrainModelPlanar.h"

namespace switched_model {

TerrainModelPlanar::TerrainModelPlanar(TerrainPlane terrainPlane) : terrainPlane_(std::move(terrainPlane)) {}

TerrainPlane TerrainModelPlanar::getLocalTerrainAtPositionInWorldAlongGravity(const vector3_t& positionInWorld) const {
  // Project point to plane to find new center, orientation stays the same
  return {projectPositionInWorldOntoPlaneAlongGravity(positionInWorld, terrainPlane_), terrainPlane_.orientationWorldToTerrain};
}

}  // namespace switched_model
