//
// Created by rgrandia on 23.06.20.
//

#pragma once

#include "ocs2_switched_model_interface/core/SwitchedModel.h"

#include "ocs2_switched_model_interface/terrain/TerrainPlane.h"

namespace switched_model {

struct ConvexTerrain {
  /// Plane coordinate defining the origin of the terrain
  TerrainPlane plane;

  /// Boundary points x-y in the terrain frame, points are order counter-clockwise. The last point is connected to the first point
  std::vector<vector2_t> boundary;
};

}