//
// Created by rgrandia on 16.03.22.
//

#include "convex_plane_decomposition/LoadGridmapFromImage.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <grid_map_cv/GridMapCvConverter.hpp>

namespace convex_plane_decomposition {

grid_map::GridMap loadGridmapFromImage(const std::string& filePath, const std::string& elevationLayer, const std::string& frameId,
                                       double resolution, double scale) {
  // Read the file
  cv::Mat image;
  image = cv::imread(filePath, cv::ImreadModes::IMREAD_GRAYSCALE);

  // Check for invalid input
  if (!image.data) {
    throw std::runtime_error("Could not open or find the image");
  }

  // Min max values
  double minValue, maxValue;
  cv::minMaxLoc(image, &minValue, &maxValue);

  grid_map::GridMap mapOut({elevationLayer});
  mapOut.setFrameId(frameId);
  grid_map::GridMapCvConverter::initializeFromImage(image, resolution, mapOut, grid_map::Position(0.0, 0.0));
  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(image, elevationLayer, mapOut, float(0.0), float(scale), 0.5);
  return mapOut;
}

}  // namespace convex_plane_decomposition