//
// Created by rgrandia on 21.04.20.
//

#pragma once

#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/terrain/ConvexTerrain.h"
#include "ocs2_switched_model_interface/terrain/TerrainPlane.h"

namespace switched_model {

class TerrainModel {
 public:
  TerrainModel() = default;
  virtual ~TerrainModel() = default;
  TerrainModel(const TerrainModel&) = delete;
  TerrainModel& operator=(const TerrainModel&) = delete;

  virtual TerrainPlane getLocalTerrainAtPositionInWorld(const vector3_t& positionInWorld) const = 0;

  virtual ConvexTerrain getConvexTerrainAtPositionInWorld(const vector3_t& positionInWorld) const {
    return {getLocalTerrainAtPositionInWorld(positionInWorld), {}};
  };
};

}