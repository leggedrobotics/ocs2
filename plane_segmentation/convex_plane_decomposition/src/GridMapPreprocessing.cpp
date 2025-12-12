#include "convex_plane_decomposition/GridMapPreprocessing.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include <grid_map_filters_rsl/inpainting.hpp>
#include <grid_map_filters_rsl/smoothing.hpp>

namespace convex_plane_decomposition {

GridMapPreprocessing::GridMapPreprocessing(const PreprocessingParameters& parameters) : parameters_(parameters) {}

void GridMapPreprocessing::preprocess(grid_map::GridMap& gridMap, const std::string& layer) const {
  inpaint(gridMap, layer);
  denoise(gridMap, layer);
  changeResolution(gridMap, layer);
}

void GridMapPreprocessing::denoise(grid_map::GridMap& gridMap, const std::string& layer) const {
  int kernelSize = std::max(1, std::min(parameters_.kernelSize, 5));  // must be 1, 3 or 5 for current image type, see doc of cv::medianBlur
  grid_map::smoothing::median(gridMap, layer, layer, kernelSize, 0, parameters_.numberOfRepeats);
}

void GridMapPreprocessing::changeResolution(grid_map::GridMap& gridMap, const std::string& layer) const {
  bool hasSameResolution = std::abs(gridMap.getResolution() - parameters_.resolution) < 1e-6;

  if (parameters_.resolution > 0.0 && !hasSameResolution) {
    grid_map::inpainting::resample(gridMap, layer, parameters_.resolution);
  }
}

void GridMapPreprocessing::inpaint(grid_map::GridMap& gridMap, const std::string& layer) const {
  const std::string& layerOut = "tmp";
  grid_map::inpainting::minValues(gridMap, layer, layerOut);

  gridMap.get(layer) = std::move(gridMap.get(layerOut));
  gridMap.erase(layerOut);
}

bool containsFiniteValue(const grid_map::Matrix& map) {
  for (int col = 0; col < map.cols(); ++col) {
    for (int row = 0; row < map.rows(); ++row) {
      if (std::isfinite(map(col, row))) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace convex_plane_decomposition
