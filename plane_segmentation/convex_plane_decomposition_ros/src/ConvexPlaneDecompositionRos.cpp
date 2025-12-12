#include "convex_plane_decomposition_ros/ConvexPlaneDecompositionRos.h"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_cv/GridMapCvProcessing.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <convex_plane_decomposition/PlaneDecompositionPipeline.h>
#include <convex_plane_decomposition_msgs/PlanarTerrain.h>

#include "convex_plane_decomposition_ros/MessageConversion.h"
#include "convex_plane_decomposition_ros/ParameterLoading.h"
#include "convex_plane_decomposition_ros/RosVisualizations.h"

namespace convex_plane_decomposition {

ConvexPlaneExtractionROS::ConvexPlaneExtractionROS(ros::NodeHandle& nodeHandle) : tfBuffer_(), tfListener_(tfBuffer_) {
  bool parametersLoaded = loadParameters(nodeHandle);

  if (parametersLoaded) {
    elevationMapSubscriber_ = nodeHandle.subscribe(elevationMapTopic_, 1, &ConvexPlaneExtractionROS::callback, this);
    filteredmapPublisher_ = nodeHandle.advertise<grid_map_msgs::GridMap>("filtered_map", 1);
    boundaryPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("boundaries", 1);
    insetPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("insets", 1);
    regionPublisher_ = nodeHandle.advertise<convex_plane_decomposition_msgs::PlanarTerrain>("planar_terrain", 1);
  }
}

ConvexPlaneExtractionROS::~ConvexPlaneExtractionROS() {
  if (callbackTimer_.getNumTimedIntervals() > 0 && planeDecompositionPipeline_ != nullptr) {
    std::stringstream infoStream;
    infoStream << "\n########################################################################\n";
    infoStream << "The benchmarking is computed over " << callbackTimer_.getNumTimedIntervals() << " iterations. \n";
    infoStream << "PlaneExtraction Benchmarking    : Average time [ms], Max time [ms]\n";
    auto printLine = [](std::string name, const Timer& timer) {
      std::stringstream ss;
      ss << std::fixed << std::setprecision(2);
      ss << "\t" << name << "\t: " << std::setw(17) << timer.getAverageInMilliseconds() << ", " << std::setw(13)
         << timer.getMaxIntervalInMilliseconds() << "\n";
      return ss.str();
    };
    infoStream << printLine("Pre-process        ", planeDecompositionPipeline_->getPrepocessTimer());
    infoStream << printLine("Sliding window     ", planeDecompositionPipeline_->getSlidingWindowTimer());
    infoStream << printLine("Contour extraction ", planeDecompositionPipeline_->getContourExtractionTimer());
    infoStream << printLine("Post-process       ", planeDecompositionPipeline_->getPostprocessTimer());
    infoStream << printLine("Total callback     ", callbackTimer_);
    std::cerr << infoStream.str() << std::endl;
  }
}

bool ConvexPlaneExtractionROS::loadParameters(const ros::NodeHandle& nodeHandle) {
  if (!nodeHandle.getParam("elevation_topic", elevationMapTopic_)) {
    ROS_ERROR("[ConvexPlaneExtractionROS] Could not read parameter `elevation_topic`.");
    return false;
  }
  if (!nodeHandle.getParam("target_frame_id", targetFrameId_)) {
    ROS_ERROR("[ConvexPlaneExtractionROS] Could not read parameter `target_frame_id`.");
    return false;
  }
  if (!nodeHandle.getParam("height_layer", elevationLayer_)) {
    ROS_ERROR("[ConvexPlaneExtractionROS] Could not read parameter `height_layer`.");
    return false;
  }
  if (!nodeHandle.getParam("submap/width", subMapWidth_)) {
    ROS_ERROR("[ConvexPlaneExtractionROS] Could not read parameter `submap/width`.");
    return false;
  }
  if (!nodeHandle.getParam("submap/length", subMapLength_)) {
    ROS_ERROR("[ConvexPlaneExtractionROS] Could not read parameter `submap/length`.");
    return false;
  }
  if (!nodeHandle.getParam("publish_to_controller", publishToController_)) {
    ROS_ERROR("[ConvexPlaneExtractionROS] Could not read parameter `publish_to_controller`.");
    return false;
  }

  PlaneDecompositionPipeline::Config config;
  config.preprocessingParameters = loadPreprocessingParameters(nodeHandle, "preprocessing/");
  config.contourExtractionParameters = loadContourExtractionParameters(nodeHandle, "contour_extraction/");
  config.ransacPlaneExtractorParameters = loadRansacPlaneExtractorParameters(nodeHandle, "ransac_plane_refinement/");
  config.slidingWindowPlaneExtractorParameters = loadSlidingWindowPlaneExtractorParameters(nodeHandle, "sliding_window_plane_extractor/");
  config.postprocessingParameters = loadPostprocessingParameters(nodeHandle, "postprocessing/");

  planeDecompositionPipeline_ = std::make_unique<PlaneDecompositionPipeline>(config);

  return true;
}

void ConvexPlaneExtractionROS::callback(const grid_map_msgs::GridMap& message) {
  callbackTimer_.startTimer();

  // Convert message to map.
  grid_map::GridMap messageMap;
  std::vector<std::string> layers{elevationLayer_};
  grid_map::GridMapRosConverter::fromMessage(message, messageMap, layers, false, false);
  if (!containsFiniteValue(messageMap.get(elevationLayer_))) {
    ROS_WARN("[ConvexPlaneExtractionROS] map does not contain any values");
    callbackTimer_.endTimer();
    return;
  }

  // Transform map if necessary
  if (targetFrameId_ != messageMap.getFrameId()) {
    std::string errorMsg;
    ros::Time timeStamp = ros::Time(0);  // Use Time(0) to get the latest transform.
    if (tfBuffer_.canTransform(targetFrameId_, messageMap.getFrameId(), timeStamp, &errorMsg)) {
      const auto transform = getTransformToTargetFrame(messageMap.getFrameId(), timeStamp);
      messageMap = grid_map::GridMapCvProcessing::getTransformedMap(std::move(messageMap), transform, elevationLayer_, targetFrameId_);
    } else {
      ROS_ERROR_STREAM("[ConvexPlaneExtractionROS] " << errorMsg);
      callbackTimer_.endTimer();
      return;
    }
  }

  // Extract submap
  bool success;
  const grid_map::Position submapPosition = [&]() {
    // The map center might be between cells. Taking the submap there can result in changing submap dimensions.
    // project map center to an index and index to center s.t. we get the location of a cell.
    grid_map::Index centerIndex;
    grid_map::Position centerPosition;
    messageMap.getIndex(messageMap.getPosition(), centerIndex);
    messageMap.getPosition(centerIndex, centerPosition);
    return centerPosition;
  }();
  grid_map::GridMap elevationMap = messageMap.getSubmap(submapPosition, Eigen::Array2d(subMapLength_, subMapWidth_), success);
  if (!success) {
    ROS_WARN("[ConvexPlaneExtractionROS] Could not extract submap");
    callbackTimer_.endTimer();
    return;
  }
  const grid_map::Matrix elevationRaw = elevationMap.get(elevationLayer_);

  // Run pipeline.
  planeDecompositionPipeline_->update(std::move(elevationMap), elevationLayer_);
  auto& planarTerrain = planeDecompositionPipeline_->getPlanarTerrain();

  // Publish terrain
  if (publishToController_) {
    regionPublisher_.publish(toMessage(planarTerrain));
  }

  // --- Visualize in Rviz --- Not published to the controller
  // Add raw map
  planarTerrain.gridMap.add("elevation_raw", elevationRaw);

  // Add segmentation
  planarTerrain.gridMap.add("segmentation");
  planeDecompositionPipeline_->getSegmentation(planarTerrain.gridMap.get("segmentation"));

  grid_map_msgs::GridMap outputMessage;
  grid_map::GridMapRosConverter::toMessage(planarTerrain.gridMap, outputMessage);
  filteredmapPublisher_.publish(outputMessage);

  const double lineWidth = 0.005;  // [m] RViz marker size
  boundaryPublisher_.publish(convertBoundariesToRosMarkers(planarTerrain.planarRegions, planarTerrain.gridMap.getFrameId(),
                                                           planarTerrain.gridMap.getTimestamp(), lineWidth));
  insetPublisher_.publish(convertInsetsToRosMarkers(planarTerrain.planarRegions, planarTerrain.gridMap.getFrameId(),
                                                    planarTerrain.gridMap.getTimestamp(), lineWidth));

  callbackTimer_.endTimer();
}

Eigen::Isometry3d ConvexPlaneExtractionROS::getTransformToTargetFrame(const std::string& sourceFrame, const ros::Time& time) {
  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer_.lookupTransform(targetFrameId_, sourceFrame, time);
  } catch (tf2::TransformException& ex) {
    ROS_ERROR("[ConvexPlaneExtractionROS] %s", ex.what());
    return Eigen::Isometry3d::Identity();
  }

  Eigen::Isometry3d transformation;

  // Extract translation.
  transformation.translation().x() = transformStamped.transform.translation.x;
  transformation.translation().y() = transformStamped.transform.translation.y;
  transformation.translation().z() = transformStamped.transform.translation.z;

  // Extract rotation.
  Eigen::Quaterniond rotationQuaternion(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                                        transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);
  transformation.linear() = rotationQuaternion.toRotationMatrix();
  return transformation;
}

}  // namespace convex_plane_decomposition
