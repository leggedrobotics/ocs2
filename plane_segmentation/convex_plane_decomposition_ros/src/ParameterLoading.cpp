//
// Created by rgrandia on 10.06.20.
//

#include "convex_plane_decomposition_ros/ParameterLoading.h"

#include <cmath>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace convex_plane_decomposition {

namespace {

std::string sanitizePrefix(const rclcpp::Node* node, const std::string& prefix) {
  std::string sanitized = prefix;

  // Convert ROS 1-style "/ns/param/" to ROS 2-style "ns.param.".
  while (!sanitized.empty() && sanitized.front() == '/') {
    sanitized.erase(sanitized.begin());
  }
  while (!sanitized.empty() && sanitized.back() == '/') {
    sanitized.pop_back();
  }
  for (char& c : sanitized) {
    if (c == '/') {
      c = '.';
    }
  }
  if (!sanitized.empty() && sanitized.back() != '.') {
    sanitized.push_back('.');
  }

  // The OCS2 perceptive demo passes the node name in the prefix (ROS 1 global params).
  // In ROS 2 parameters are node-local, so we drop it if present.
  if (node != nullptr) {
    const std::string nodeName = node->get_name();
    const std::string nodePrefix = nodeName + ".";
    if (sanitized.rfind(nodePrefix, 0) == 0) {
      sanitized.erase(0, nodePrefix.size());
    }
  }

  return sanitized;
}

template <typename T>
void loadParameter(rclcpp::Node* node, const std::string& prefix, const std::string& param, T& value) {
  if (node == nullptr) {
    throw std::invalid_argument("loadParameter(): node is nullptr");
  }

  const std::string fullName = sanitizePrefix(node, prefix) + param;
  if (!node->has_parameter(fullName)) {
    node->declare_parameter(fullName, value);
  }
  node->get_parameter(fullName, value);
}

}  // namespace

PreprocessingParameters loadPreprocessingParameters(rclcpp::Node* node, const std::string& prefix) {
  PreprocessingParameters preprocessingParameters;
  loadParameter(node, prefix, "resolution", preprocessingParameters.resolution);
  loadParameter(node, prefix, "kernelSize", preprocessingParameters.kernelSize);
  loadParameter(node, prefix, "numberOfRepeats", preprocessingParameters.numberOfRepeats);
  return preprocessingParameters;
}

contour_extraction::ContourExtractionParameters loadContourExtractionParameters(rclcpp::Node* node, const std::string& prefix) {
  contour_extraction::ContourExtractionParameters contourParams;
  loadParameter(node, prefix, "marginSize", contourParams.marginSize);
  return contourParams;
}

ransac_plane_extractor::RansacPlaneExtractorParameters loadRansacPlaneExtractorParameters(rclcpp::Node* node, const std::string& prefix) {
  ransac_plane_extractor::RansacPlaneExtractorParameters ransacParams;
  loadParameter(node, prefix, "probability", ransacParams.probability);
  loadParameter(node, prefix, "min_points", ransacParams.min_points);
  loadParameter(node, prefix, "epsilon", ransacParams.epsilon);
  loadParameter(node, prefix, "cluster_epsilon", ransacParams.cluster_epsilon);
  loadParameter(node, prefix, "normal_threshold", ransacParams.normal_threshold);
  return ransacParams;
}

sliding_window_plane_extractor::SlidingWindowPlaneExtractorParameters loadSlidingWindowPlaneExtractorParameters(
    rclcpp::Node* node, const std::string& prefix) {
  sliding_window_plane_extractor::SlidingWindowPlaneExtractorParameters swParams;
  loadParameter(node, prefix, "kernel_size", swParams.kernel_size);
  loadParameter(node, prefix, "planarity_opening_filter", swParams.planarity_opening_filter);

  double planeInclinationThresholdDegrees = std::acos(swParams.plane_inclination_threshold) * 180.0 / M_PI;
  loadParameter(node, prefix, "plane_inclination_threshold_degrees", planeInclinationThresholdDegrees);
  swParams.plane_inclination_threshold = std::cos(planeInclinationThresholdDegrees * M_PI / 180.0);

  double localPlaneInclinationThresholdDegrees = std::acos(swParams.local_plane_inclination_threshold) * 180.0 / M_PI;
  loadParameter(node, prefix, "local_plane_inclination_threshold_degrees", localPlaneInclinationThresholdDegrees);
  swParams.local_plane_inclination_threshold = std::cos(localPlaneInclinationThresholdDegrees * M_PI / 180.0);

  loadParameter(node, prefix, "plane_patch_error_threshold", swParams.plane_patch_error_threshold);
  loadParameter(node, prefix, "min_number_points_per_label", swParams.min_number_points_per_label);
  loadParameter(node, prefix, "connectivity", swParams.connectivity);
  loadParameter(node, prefix, "include_ransac_refinement", swParams.include_ransac_refinement);
  loadParameter(node, prefix, "global_plane_fit_distance_error_threshold", swParams.global_plane_fit_distance_error_threshold);
  loadParameter(node, prefix, "global_plane_fit_angle_error_threshold_degrees", swParams.global_plane_fit_angle_error_threshold_degrees);
  return swParams;
}

PostprocessingParameters loadPostprocessingParameters(rclcpp::Node* node, const std::string& prefix) {
  PostprocessingParameters postprocessingParameters;
  loadParameter(node, prefix, "extracted_planes_height_offset", postprocessingParameters.extracted_planes_height_offset);
  loadParameter(node, prefix, "nonplanar_height_offset", postprocessingParameters.nonplanar_height_offset);
  loadParameter(node, prefix, "nonplanar_horizontal_offset", postprocessingParameters.nonplanar_horizontal_offset);
  loadParameter(node, prefix, "smoothing_dilation_size", postprocessingParameters.smoothing_dilation_size);
  loadParameter(node, prefix, "smoothing_box_kernel_size", postprocessingParameters.smoothing_box_kernel_size);
  loadParameter(node, prefix, "smoothing_gauss_kernel_size", postprocessingParameters.smoothing_gauss_kernel_size);
  return postprocessingParameters;
}

}  // namespace convex_plane_decomposition
