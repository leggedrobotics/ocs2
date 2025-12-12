#include "convex_plane_decomposition/Postprocessing.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include <grid_map_filters_rsl/processing.hpp>
#include <grid_map_filters_rsl/inpainting.hpp>

namespace convex_plane_decomposition {

Postprocessing::Postprocessing(const PostprocessingParameters& parameters) : parameters_(parameters) {}

void Postprocessing::postprocess(PlanarTerrain& planarTerrain, const std::string& elevationLayer,
                                 const std::string& planeSegmentationLayer) const {
  auto& elevationData = planarTerrain.gridMap.get(elevationLayer);
  const auto& planarityMask = planarTerrain.gridMap.get(planeSegmentationLayer);

  // Store the unaltered map
  planarTerrain.gridMap.add(elevationLayer + "_before_postprocess", elevationData);

  // post process planar regions
  addHeightOffset(planarTerrain.planarRegions);

  // Add smooth layer for base reference
  addSmoothLayer(planarTerrain.gridMap, elevationData, planarityMask);

  // post process elevation map
  dilationInNonplanarRegions(elevationData, planarityMask);
  addHeightOffset(elevationData, planarityMask);
}

void Postprocessing::dilationInNonplanarRegions(Eigen::MatrixXf& elevationData, const Eigen::MatrixXf& planarityMask) const {
  if (parameters_.nonplanar_horizontal_offset > 0) {
    // Convert to opencv image
    cv::Mat elevationImage;
    cv::eigen2cv(elevationData, elevationImage);  // creates CV_32F image

    // dilate
    const int dilationSize = 2 * parameters_.nonplanar_horizontal_offset + 1;  //
    const int dilationType = cv::MORPH_ELLIPSE;                                // ellipse inscribed in the square of size dilationSize
    const auto dilationKernel_ = cv::getStructuringElement(dilationType, cv::Size(dilationSize, dilationSize));
    cv::dilate(elevationImage, elevationImage, dilationKernel_, cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);

    // convert back
    Eigen::MatrixXf elevationDilated;
    cv::cv2eigen(elevationImage, elevationDilated);

    // merge: original elevation for planar regions (mask = 1.0), dilated elevation for non-planar (mask = 0.0)
    elevationData = planarityMask.array() * elevationData.array() + (1.0 - planarityMask.array()) * elevationDilated.array();
  }
}

void Postprocessing::addHeightOffset(Eigen::MatrixXf& elevationData, const Eigen::MatrixXf& planarityMask) const {
  // lift elevation layer. For untraversable offset we first add the offset everywhere and substract it again in traversable regions.
  if (parameters_.extracted_planes_height_offset != 0.0 || parameters_.nonplanar_height_offset != 0.0) {
    elevationData.array() += (parameters_.extracted_planes_height_offset + parameters_.nonplanar_height_offset);

    if (parameters_.nonplanar_height_offset != 0.0) {
      elevationData.noalias() -= parameters_.nonplanar_height_offset * planarityMask;
    }
  }
}

void Postprocessing::addHeightOffset(std::vector<PlanarRegion>& planarRegions) const {
  if (parameters_.extracted_planes_height_offset != 0.0) {
    for (auto& planarRegion : planarRegions) {
      planarRegion.transformPlaneToWorld.translation().z() += parameters_.extracted_planes_height_offset;
    }
  }
}

void Postprocessing::addSmoothLayer(grid_map::GridMap& gridMap, const Eigen::MatrixXf& elevationData,
                                    const Eigen::MatrixXf& planarityMask) const {
  auto kernelSizeInPixels = [res = gridMap.getResolution()](double realSize) {
    return 2 * static_cast<int>(std::round(realSize / res)) + 1;
  };

  // Helper to convert to Eigen
  auto toEigen = [](const cv::Mat& image) {
    Eigen::MatrixXf data;
    cv::cv2eigen(image, data);
    return data;
  };

  // Helper to openCV
  auto toCv = [](const Eigen::MatrixXf& data) {
    cv::Mat image;
    cv::eigen2cv(data, image);
    return image;
  };

  const int dilationSize = kernelSizeInPixels(parameters_.smoothing_dilation_size);
  const int kernel = kernelSizeInPixels(parameters_.smoothing_box_kernel_size);
  const int kernelGauss = kernelSizeInPixels(parameters_.smoothing_gauss_kernel_size);


  // Set non planar regions to NaN
  Eigen::MatrixXf elevationWithNaN =
      (planarityMask.array() == 1.0)
          .select(elevationData, Eigen::MatrixXf::Constant(elevationData.rows(), elevationData.cols(), NAN));
  gridMap.add("elevationWithNaN", elevationWithNaN);

  // Inpainting
  grid_map::inpainting::minValues(gridMap, "elevationWithNaN", "elevationWithNaN_i");

  // Closing
  auto elevationWithNaNCv = toCv(gridMap.get("elevationWithNaN_i"));
  const int dilationType = cv::MORPH_ELLIPSE;  // ellipse inscribed in the square of size dilationSize
  const auto dilationKernel_ = cv::getStructuringElement(dilationType, cv::Size(dilationSize, dilationSize));
  cv::morphologyEx(elevationWithNaNCv, elevationWithNaNCv, cv::MORPH_CLOSE, dilationKernel_, cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);

  gridMap.add("elevationWithNaNClosed", toEigen(elevationWithNaNCv));

  // Dilation with a slope of 45 deg
  int halfDilationSize = (dilationSize - 1) / 2;
  float slope = gridMap.getResolution();  // dh dpixel
  Eigen::MatrixXf offsets(dilationSize, dilationSize);
  for (int i = 0; i < dilationSize; ++i) {
    for (int j = 0; j < dilationSize; ++j) {
      const auto dx = (i - halfDilationSize);
      const auto dy = (j - halfDilationSize);
      offsets(i, j) = slope * std::sqrt(dx * dx + dy * dy);
    }
  }
  grid_map::processing::applyKernelFunction(
      gridMap, "elevationWithNaNClosed", "elevationWithNaNClosedDilated", dilationSize,
      [&](const Eigen::Ref<const grid_map::GridMap::Matrix>& data) -> float { return (data - offsets).maxCoeffOfFinites(); });

  // Convert to openCV
  auto elevationWithNaNImage = toCv(gridMap.get("elevationWithNaNClosedDilated"));

  // Filter definition
  auto smoothingFilter = [kernel, kernelGauss](const cv::Mat& imageIn) {
    cv::Mat imageOut;
    cv::boxFilter(imageIn, imageOut, -1, {kernel, kernel}, cv::Point(-1, -1), true, cv::BorderTypes::BORDER_REPLICATE);
    cv::GaussianBlur(imageOut, imageOut, {kernelGauss, kernelGauss}, 0, 0, cv::BorderTypes::BORDER_REPLICATE);
    return imageOut;
  };
  auto smooth_planar = smoothingFilter(elevationWithNaNImage);

  // Add layer to map.
  gridMap.add("smooth_planar", toEigen(smooth_planar));
}

}  // namespace convex_plane_decomposition
