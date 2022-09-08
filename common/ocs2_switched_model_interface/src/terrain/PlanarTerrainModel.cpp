//
// Created by rgrandia on 29.04.20.
//

#include "ocs2_switched_model_interface/terrain/PlanarTerrainModel.h"

namespace switched_model {

PlanarTerrainModel::PlanarTerrainModel(TerrainPlane terrainPlane) : terrainPlane_(terrainPlane), sdf_(std::move(terrainPlane)) {}

TerrainPlane PlanarTerrainModel::getLocalTerrainAtPositionInWorldAlongGravity(
    const vector3_t& positionInWorld, std::function<scalar_t(const vector3_t&)> penaltyFunction) const {
  // Project point to plane to find new center, orientation stays the same
  return {projectPositionInWorldOntoPlaneAlongGravity(positionInWorld, terrainPlane_), terrainPlane_.orientationWorldToTerrain};
}

vector3_t PlanarTerrainModel::getHighestObstacleAlongLine(const vector3_t& position1InWorld, const vector3_t& position2InWorld) const {
  // The highest point on a plane is at the end of the line
  const auto projection1 = projectPositionInWorldOntoPlaneAlongGravity(position1InWorld, terrainPlane_);
  const auto projection2 = projectPositionInWorldOntoPlaneAlongGravity(position2InWorld, terrainPlane_);

  if (projection1.z() > projection2.z()) {
    return projection1;
  } else {
    return projection2;
  }
}

std::vector<vector2_t> PlanarTerrainModel::getHeightProfileAlongLine(const vector3_t& position1InWorld,
                                                                     const vector3_t& position2InWorld) const {
  // Provide end points and one middle point as the height profile.
  const auto projection1 = projectPositionInWorldOntoPlaneAlongGravity(position1InWorld, terrainPlane_);
  const auto projection2 = projectPositionInWorldOntoPlaneAlongGravity(position2InWorld, terrainPlane_);
  return {{0.0, projection1.z()}, {0.5, 0.5 * (projection1.z() + projection2.z())}, {1.0, projection2.z()}};
}

}  // namespace switched_model
