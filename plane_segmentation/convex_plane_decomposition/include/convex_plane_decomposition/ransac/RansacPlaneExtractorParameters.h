//
// Created by rgrandia on 07.06.20.
//

#pragma once

namespace ransac_plane_extractor {

struct RansacPlaneExtractorParameters {
  /// Set probability to miss the largest primitive at each iteration.
  double probability = 0.01;
  /// Detect shapes with at least N points.
  double min_points = 4;
  /// [m] Set maximum Euclidean distance between a point and a shape.
  double epsilon = 0.025;
  /// Set maximum Euclidean distance between points to be clustered. Two points are connected if separated by a distance of at most
  /// 2*sqrt(2)*cluster_epsilon = 2.828 * cluster_epsilon
  double cluster_epsilon = 0.08;
  /// [deg] Set maximum normal deviation between cluster surface_normal and point normal.
  double normal_threshold = 25.0;
};

}  // namespace ransac_plane_extractor
