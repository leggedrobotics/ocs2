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

inline int getNextVertex(int i, size_t N) {
  int next = i + 1;
  return (next < N) ? next : 0;  // next point with wrap around
}

inline int getPreviousVertex(int i, size_t N) {
  return (i > 0) ? (i - 1) : (N - 1);  // previous point with wrap around
}

}  // namespace switched_model
