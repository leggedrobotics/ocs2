//
// Created by rgrandia on 08.09.22.
//

#include "ocs2_switched_model_interface/terrain/PlanarSignedDistanceField.h"

namespace switched_model {

PlanarSignedDistanceField::PlanarSignedDistanceField(TerrainPlane terrainPlane) : terrainPlane_(std::move(terrainPlane)) {}

PlanarSignedDistanceField::PlanarSignedDistanceField(const PlanarSignedDistanceField& other) : terrainPlane_(other.terrainPlane_) {}

scalar_t PlanarSignedDistanceField::value(const vector3_t& position) const {
  return terrainSignedDistanceFromPositionInWorld(position, terrainPlane_);
}

vector3_t PlanarSignedDistanceField::derivative(const vector3_t& position) const {
  return surfaceNormalInWorld(terrainPlane_);
}

std::pair<scalar_t, vector3_t> PlanarSignedDistanceField::valueAndDerivative(const vector3_t& position) const {
  return {value(position), derivative(position)};
}

}  // namespace switched_model
