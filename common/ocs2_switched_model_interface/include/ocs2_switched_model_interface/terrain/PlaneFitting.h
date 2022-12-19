//
// Created by rgrandia on 27.11.20.
//

#pragma once

#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/terrain/TerrainPlane.h"

namespace switched_model {

NormalAndPosition estimatePlane(const std::vector<vector3_t>& regressionPoints);

}  // namespace switched_model
