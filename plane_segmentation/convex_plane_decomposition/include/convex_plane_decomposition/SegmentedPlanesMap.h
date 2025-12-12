//
// Created by rgrandia on 10.06.20.
//

#pragma once

#include <Eigen/Core>
#include <opencv2/core/mat.hpp>

#include "PlanarRegion.h"

namespace convex_plane_decomposition {

struct SegmentedPlanesMap {
  /// Unordered collection of all labels and corresponding plane parameters
  std::vector<std::pair<int, NormalAndPosition>> labelPlaneParameters;

  /// Image with a each pixel being assigned and integer value corresponding to the label. Might contain labels that are not in
  /// labelPlaneParameters. These labels should be seen as background.
  cv::Mat labeledImage;

  /// Size of each pixel [m]
  double resolution;

  /// World X-Y position [m] of the (0, 0) pixel in the image.
  Eigen::Vector2d mapOrigin;

  /// All label values are smaller than or equal to highestLabel
  int highestLabel;
};

}  // namespace convex_plane_decomposition
