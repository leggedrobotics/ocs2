/**
 * @file        inpainting.cpp
 * @authors     Fabian Jenelten
 * @date        18.05, 2021
 * @affiliation ETH RSL
 * @brief      Inpainting filter (extrapolate nan values from surrounding data).
 */

// grid map filters rsl.
#include <grid_map_filters_rsl/inpainting.hpp>
#include <grid_map_filters_rsl/processing.hpp>

// open cv.
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/photo.hpp>

// stl.
#include <limits>

namespace grid_map {
namespace inpainting {

void minValues(grid_map::GridMap& map, const std::string& layerIn, const std::string& layerOut) {
  // Create new layer if missing
  if (!map.exists(layerOut)) {
    map.add(layerOut, map.get(layerIn));
  }

  // Reference to in, and out maps, initialize with copy
  const grid_map::Matrix& H_in = map.get(layerIn);
  grid_map::Matrix& H_out = map.get(layerOut);
  H_out = H_in;

  // Some constant
  const int numCols = H_in.cols();
  const int maxColId = numCols - 1;
  const int numRows = H_in.rows();
  const int maxRowId = numRows - 1;

  // Common operation of updating the minimum and keeping track if the minimum was updated.
  auto compareAndStoreMin = [](float newValue, float& currentMin, bool& changedValue) {
    if (!std::isnan(newValue)) {
      if (newValue < currentMin || std::isnan(currentMin)) {
        currentMin = newValue;
        changedValue = true;
      }
    }
  };

  /*
   * Fill each cell that needs inpainting with the min of its neighbours until the map doesn't change anymore.
   * This way each inpainted area gets the minimum value along its contour.
   *
   * We will be reading and writing to H_out during iteration. However, the aliasing does not break the correctness of the algorithm.
   */
  bool hasAtLeastOneValue = true;
  bool changedValue = true;
  while (changedValue && hasAtLeastOneValue) {
    hasAtLeastOneValue = false;
    changedValue = false;
    for (int colId = 0; colId < numCols; ++colId) {
      for (int rowId = 0; rowId < numRows; ++rowId) {
        if (std::isnan(H_in(rowId, colId))) {
          auto& middleValue = H_out(rowId, colId);

          // left
          if (colId > 0) {
            const auto leftValue = H_out(rowId, colId - 1);
            compareAndStoreMin(leftValue, middleValue, changedValue);
          }
          // right
          if (colId < maxColId) {
            const auto rightValue = H_out(rowId, colId + 1);
            compareAndStoreMin(rightValue, middleValue, changedValue);
          }
          // top
          if (rowId > 0) {
            const auto topValue = H_out(rowId - 1, colId);
            compareAndStoreMin(topValue, middleValue, changedValue);
          }
          // bottom
          if (rowId < maxRowId) {
            const auto bottomValue = H_out(rowId + 1, colId);
            compareAndStoreMin(bottomValue, middleValue, changedValue);
          }
        } else {
          hasAtLeastOneValue = true;
        }
      }
    }
  }
}

void biLinearInterpolation(grid_map::GridMap& map, const std::string& layerIn, const std::string& layerOut) {
  // Create new layer if missing
  if (!map.exists(layerOut)) {
    map.add(layerOut, map.get(layerIn));
  } else {
    // initialize with a copy
    map.get(layerOut) = map.get(layerIn);
  }

  // Helper variables.
  std::array<Eigen::Vector2i, 4> indices;
  std::array<float, 4> values;
  Eigen::Matrix4f A;
  Eigen::Vector4f b;
  A.setOnes();
  Eigen::Vector4f weights;
  bool success = true;
  constexpr auto infinity = std::numeric_limits<float>::max();

  // Init.
  std::fill(values.begin(), values.end(), NAN);
  std::fill(indices.begin(), indices.end(), Eigen::Vector2i(0, 0));

  // Reference to in and out maps.
  const grid_map::Matrix& H_in = map.get(layerIn);
  grid_map::Matrix& H_out = map.get(layerOut);

  for (auto colId = 0; colId < H_in.cols(); ++colId) {
    for (auto rowId = 0; rowId < H_in.rows(); ++rowId) {
      if (std::isnan(H_in(rowId, colId))) {
        // Note: if we don't find a valid neighbour, we use the previous index-value pair.
        auto minValue = infinity;
        const Eigen::Vector2i index0(rowId, colId);

        // Search in negative direction.
        for (auto id = rowId - 1; id >= 0; --id) {
          auto newValue = H_in(id, colId);
          if (!std::isnan(newValue)) {
            indices[0] = Eigen::Vector2i(id, colId);
            values[0] = newValue;
            minValue = std::fmin(minValue, newValue);
            break;
          }
        }

        for (auto id = colId - 1; id >= 0; --id) {
          auto newValue = H_in(rowId, id);
          if (!std::isnan(newValue)) {
            indices[1] = Eigen::Vector2i(rowId, id);
            values[1] = newValue;
            minValue = std::fmin(minValue, newValue);
            break;
          }
        }

        // Search in positive direction.
        for (auto id = rowId + 1; id < H_in.rows(); ++id) {
          auto newValue = H_in(id, colId);
          if (!std::isnan(newValue)) {
            indices[2] = Eigen::Vector2i(id, colId);
            values[2] = newValue;
            minValue = std::fmin(minValue, newValue);
            break;
          }
        }

        for (auto id = colId + 1; id < H_in.cols(); ++id) {
          auto newValue = H_in(rowId, id);
          if (!std::isnan(newValue)) {
            indices[3] = Eigen::Vector2i(rowId, id);
            values[3] = newValue;
            minValue = std::fmin(minValue, newValue);
            break;
          }
        }

        // Cannot interpolate if there are not 4 corner points.
        if (std::any_of(values.begin(), values.end(), [](float value) { return std::isnan(value); })) {
          if (minValue < infinity) {
            H_out(rowId, colId) = minValue;
          } else {
            success = false;
          }
          continue;
        }

        // Interpolation weights (https://en.wikipedia.org/wiki/Bilinear_interpolation).
        for (auto id = 0U; id < 4U; ++id) {
          A(id, 1U) = static_cast<float>(indices[id].x());
          A(id, 2U) = static_cast<float>(indices[id].y());
          A(id, 3U) = static_cast<float>(indices[id].x() * indices[id].y());
          b(id) = values[id];
        }
        weights = A.colPivHouseholderQr().solve(b);

        // Value according to bi-linear interpolation.
        H_out(rowId, colId) = weights.dot(Eigen::Vector4f(1.0, static_cast<float>(index0.x()), static_cast<float>(index0.y()),
                                                          static_cast<float>(index0.x() * index0.y())));
      }
    }
  }

  // If failed, try again.
  if (!success) {
    map.get(layerIn) = map.get(layerOut);
    return nonlinearInterpolation(map, layerIn, layerOut, 2. * map.getResolution());
  }
}

void nonlinearInterpolation(grid_map::GridMap& map, const std::string& layerIn, const std::string& layerOut, double inpaintRadius) {
  // Create new layer if missing.
  if (!map.exists(layerOut)) {
    map.add(layerOut);
  }

  // Reference to in map.
  const grid_map::Matrix& H_in = map.get(layerIn);

  // Create mask.
  Eigen::Matrix<uchar, -1, -1> mask = H_in.unaryExpr([](float val) { return (std::isnan(val)) ? uchar(1) : uchar(0); });
  cv::Mat maskImage;
  cv::eigen2cv(mask, maskImage);

  // Convert grid map to image.
  cv::Mat elevationImageIn;
  const float minValue = H_in.minCoeffOfFinites();
  const float maxValue = H_in.maxCoeffOfFinites();
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, layerIn, CV_8UC1, minValue, maxValue, elevationImageIn);

  // Inpainting.
  cv::Mat elevationImageOut;
  const double radiusInPixels = inpaintRadius / map.getResolution();
  cv::inpaint(elevationImageIn, maskImage, elevationImageOut, radiusInPixels, cv::INPAINT_NS);

  // Get inpainting as float.
  cv::Mat filledImageFloat;
  constexpr float maxUCharValue = 255.F;
  elevationImageOut.convertTo(filledImageFloat, CV_32F, (maxValue - minValue) / maxUCharValue, minValue);

  // Copy inpainted values back to elevation map.
  cv::Mat elevationImageFloat;
  cv::eigen2cv(H_in, elevationImageFloat);
  filledImageFloat.copyTo(elevationImageFloat, maskImage);

  // Set new output layer.
  cv::cv2eigen(elevationImageFloat, map.get(layerOut));
}

void resample(grid_map::GridMap& map, const std::string& layer, double newRes) {
  // Original map info
  const auto oldPos = map.getPosition();
  const auto oldSize = map.getSize();
  const auto oldRes = map.getResolution();

  if (oldRes == newRes) {
    return;
  }

  // Layers to be resampled.
  std::vector<std::string> layer_names;
  if (layer == "all") {
    layer_names = map.getLayers();
  } else {
    layer_names.push_back(layer);
  }

  for (const auto& layer_name : layer_names) {
    Eigen::MatrixXf elevationMap = std::move(map.get(layer_name));

    // Convert elevation map to open-cv image.
    cv::Mat elevationImage;
    cv::eigen2cv(elevationMap, elevationImage);

    // Compute new dimensions.
    const double scaling = oldRes / newRes;
    int width = int(elevationImage.size[1] * scaling);
    int height = int(elevationImage.size[0] * scaling);
    cv::Size dim{width, height};

    // Resize image
    cv::Mat resizedImage;
    cv::resize(elevationImage, resizedImage, dim, 0, 0, cv::INTER_LINEAR);
    cv::cv2eigen(resizedImage, elevationMap);

    // Compute true new resolution. Might be slightly different due to rounding. Take average of both dimensions.
    grid_map::Size newSize = {elevationMap.rows(), elevationMap.cols()};
    newRes = 0.5 * ((oldSize[0] * oldRes) / newSize[0] + (oldSize[1] * oldRes) / newSize[1]);

    // Store new map.
    map.setGeometry({newSize[0] * newRes, newSize[1] * newRes}, newRes, oldPos);
    map.get(layer_name) = std::move(elevationMap);
  }
}
}  // namespace inpainting
}  // namespace grid_map
