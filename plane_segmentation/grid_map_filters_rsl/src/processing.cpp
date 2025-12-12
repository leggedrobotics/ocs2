/**
 * @file        processing.cpp
 * @authors     Fabian Jenelten
 * @date        04.08, 2021
 * @affiliation ETH RSL
 * @brief       Processing filter (everything that is not smoothing or inpainting).
 */

// grid map filters rsl.
#include <grid_map_filters_rsl/processing.hpp>

namespace grid_map {
namespace processing {

void dilate(grid_map::GridMap& map, const std::string& layerIn, const std::string& layerOut, const grid_map::Matrix& mask, int kernelSize,
            bool inpaint) {
  // Create new layer if missing
  if (!map.exists(layerOut)) {
    map.add(layerOut, map.get(layerIn));
  }

  // Reference to in and out maps.
  const grid_map::Matrix& H_in = map.get(layerIn);
  grid_map::Matrix& H_out = map.get(layerOut);

  // Apply mask.
  grid_map::Matrix H_in_masked;
  if (mask.cols() == 0 || mask.rows() == 0) {
    H_in_masked = H_in;
  } else {
    H_in_masked = H_in.cwiseProduct(mask);
  }
  const auto maxKernelId = (kernelSize - 1) / 2;

  for (auto colId = 0; colId < H_in.cols(); ++colId) {
    for (auto rowId = 0; rowId < H_in.rows(); ++rowId) {
      // Move index to the korner of the kernel window.
      auto cornerColId = std::max(colId - maxKernelId, 0);
      auto cornerRowId = std::max(rowId - maxKernelId, 0);

      // Make sure we don't overshoot.
      if (cornerColId + kernelSize > H_in.cols()) {
        cornerColId = H_in.cols() - kernelSize;
      }

      if (cornerRowId + kernelSize > H_in.rows()) {
        cornerRowId = H_in.rows() - kernelSize;
      }

      // Find maximum in region.
      if (inpaint || !std::isnan(H_in(rowId, colId))) {
        const auto maxInKernel = H_in_masked.block(cornerRowId, cornerColId, kernelSize, kernelSize).maxCoeffOfFinites();
        H_out(rowId, colId) = std::isnan(maxInKernel) ? H_in(rowId, colId) : maxInKernel;
      } else {
        H_out(rowId, colId) = NAN;
      }
    }
  }
}

void erode(grid_map::GridMap& map, const std::string& layerIn, const std::string& layerOut, const grid_map::Matrix& mask, int kernelSize,
           bool inpaint) {
  // Create new layer if missing
  if (!map.exists(layerOut)) {
    map.add(layerOut, map.get(layerIn));
  }

  // Reference to in and out maps.
  const grid_map::Matrix& H_in = map.get(layerIn);
  grid_map::Matrix& H_out = map.get(layerOut);

  // Apply mask.
  grid_map::Matrix H_in_masked;
  if (mask.cols() == 0 || mask.rows() == 0) {
    H_in_masked = H_in;
  } else {
    H_in_masked = H_in.cwiseProduct(mask);
  }
  const auto maxKernelId = (kernelSize - 1) / 2;

  for (auto colId = 0; colId < H_in.cols(); ++colId) {
    for (auto rowId = 0; rowId < H_in.rows(); ++rowId) {
      // Move index to the korner of the kernel window.
      auto cornerColId = std::max(colId - maxKernelId, 0);
      auto cornerRowId = std::max(rowId - maxKernelId, 0);

      // Make sure we don't overshoot.
      if (cornerColId + kernelSize > H_in.cols()) {
        cornerColId = H_in.cols() - kernelSize;
      }

      if (cornerRowId + kernelSize > H_in.rows()) {
        cornerRowId = H_in.rows() - kernelSize;
      }

      // Find minimum in region.
      if (inpaint || !std::isnan(H_in(rowId, colId))) {
        const auto minInKernel = H_in_masked.block(cornerRowId, cornerColId, kernelSize, kernelSize).minCoeffOfFinites();
        H_out(rowId, colId) = std::isnan(minInKernel) ? H_in(rowId, colId) : minInKernel;
      } else {
        H_out(rowId, colId) = NAN;
      }
    }
  }
}

void outline(grid_map::GridMap& map, const std::string& layerIn, const std::string& layerOut) {
  // Create new layer if missing
  if (!map.exists(layerOut)) {
    map.add(layerOut, map.get(layerIn));
  }

  // Reference to in and out maps.
  const grid_map::Matrix& H_in = map.get(layerIn);
  grid_map::Matrix& H_out = map.get(layerOut);

  constexpr auto kernelSize = 3;
  constexpr auto maxKernelId = (kernelSize - 1) / 2;

  for (auto colId = 0; colId < H_in.cols(); ++colId) {
    for (auto rowId = 0; rowId < H_in.rows(); ++rowId) {
      // Move index to the korner of the kernel window.
      auto cornerColId = std::max(colId - maxKernelId, 0);
      auto cornerRowId = std::max(rowId - maxKernelId, 0);

      // Make sure we don't overshoot.
      if (cornerColId + kernelSize > H_in.cols()) {
        cornerColId = H_in.cols() - kernelSize;
      }

      if (cornerRowId + kernelSize > H_in.rows()) {
        cornerRowId = H_in.rows() - kernelSize;
      }

      // Check if grid cell touches the nan grid cell.
      if (H_in.block(cornerRowId, cornerColId, kernelSize, kernelSize).hasNaN()) {
        H_out(rowId, colId) = H_in(rowId, colId);
      } else {
        H_out(rowId, colId) = NAN;
      }
    }
  }
}

void applyKernelFunction(grid_map::GridMap& map, const std::string& layerIn, const std::string& layerOut, int kernelSize,
                         std::function<float(const Eigen::Ref<const grid_map::GridMap::Matrix>&)> func) {
  // Create new layer if missing
  if (!map.exists(layerOut)) {
    map.add(layerOut, map.get(layerIn));
  }

  // Reference to in and out maps.
  const grid_map::Matrix& H_in = map.get(layerIn);
  grid_map::Matrix& H_out = map.get(layerOut);

  const auto maxKernelId = (kernelSize - 1) / 2;

  for (auto colId = 0; colId < H_in.cols(); ++colId) {
    for (auto rowId = 0; rowId < H_in.rows(); ++rowId) {
      // Move index to the korner of the kernel window.
      auto cornerColId = std::max(colId - maxKernelId, 0);
      auto cornerRowId = std::max(rowId - maxKernelId, 0);

      // Make sure we don't overshoot.
      if (cornerColId + kernelSize > H_in.cols()) {
        cornerColId = H_in.cols() - kernelSize;
      }

      if (cornerRowId + kernelSize > H_in.rows()) {
        cornerRowId = H_in.rows() - kernelSize;
      }

      // Apply user defined function
      H_out(rowId, colId) = func(H_in.block(cornerRowId, cornerColId, kernelSize, kernelSize));
    }
  }
}

}  // namespace processing
}  // namespace grid_map
