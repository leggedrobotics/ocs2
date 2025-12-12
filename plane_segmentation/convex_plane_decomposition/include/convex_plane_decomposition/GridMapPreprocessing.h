#pragma once

#include <Eigen/Core>

#include <grid_map_core/GridMap.hpp>

namespace convex_plane_decomposition {

struct PreprocessingParameters {
  /// Resample to this resolution, set to negative values to skip
  double resolution = 0.04;
  /// Kernel size of the median filter, either 3 or 5
  int kernelSize = 3;
  /// Number of times the image is filtered
  int numberOfRepeats = 2;
};

class GridMapPreprocessing {
 public:
  GridMapPreprocessing(const PreprocessingParameters& parameters);

  void preprocess(grid_map::GridMap& gridMap, const std::string& layer) const;

 private:
  void denoise(grid_map::GridMap& gridMap, const std::string& layer) const;
  void changeResolution(grid_map::GridMap& gridMap, const std::string& layer) const;
  void inpaint(grid_map::GridMap& gridMap, const std::string& layer) const;

  PreprocessingParameters parameters_;
};

/**
 * @return true if any of the elements in the map are finite
 */
bool containsFiniteValue(const grid_map::Matrix& map);

}  // namespace convex_plane_decomposition
