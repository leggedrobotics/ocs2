//
// Created by rgrandia on 29.04.20.
//

#pragma once

#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/terrain/TerrainModel.h"

namespace switched_model {

class TerrainModelPlanar : public TerrainModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TerrainModelPlanar(TerrainPlane terrainPlane);
  ~TerrainModelPlanar() override = default;

  TerrainPlane getLocalTerrainAtPositionInWorld(const vector3_t& positionInWorld) const override;

 private:
  TerrainPlane terrainPlane_;
};

}