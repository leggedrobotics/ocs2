//
// Created by rgrandia on 07.06.20.
//

#pragma once

namespace convex_plane_decomposition {
namespace contour_extraction {

struct ContourExtractionParameters {
  /// Size of the kernel creating the boundary offset. In number of (sub) pixels.
  int marginSize = 1;
};

}  // namespace contour_extraction
}  // namespace convex_plane_decomposition
