//
// Created by rgrandia on 25.10.21.
//

#include <ros/ros.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <grid_map_filters_rsl/inpainting.hpp>
#include <grid_map_filters_rsl/smoothing.hpp>

double noiseUniform;
double noiseGauss;
double outlierPercentage;
bool blur;
double frequency;
std::string elevationMapTopicIn;
std::string elevationMapTopicOut;
std::string elevationLayer;
ros::Publisher publisher;
grid_map::GridMap::Matrix noiseLayer;

void createNoise(size_t row, size_t col) {
  // Box-Muller Transform
  grid_map::GridMap::Matrix u1 = 0.5 * grid_map::GridMap::Matrix::Random(row, col).array() + 0.5;
  grid_map::GridMap::Matrix u2 = 0.5 * grid_map::GridMap::Matrix::Random(row, col).array() + 0.5;
  grid_map::GridMap::Matrix gauss01 =
      u1.binaryExpr(u2, [&](float v1, float v2) { return std::sqrt(-2.0f * log(v1)) * cos(2.0f * M_PIf32 * v2); });

  noiseLayer = noiseUniform * grid_map::GridMap::Matrix::Random(row, col) + noiseGauss * gauss01;
}

void callback(const grid_map_msgs::GridMap::ConstPtr& message) {
  grid_map::GridMap messageMap;
  grid_map::GridMapRosConverter::fromMessage(*message, messageMap);

  if (blur) {
    // Copy!
    auto originalMap = messageMap.get(elevationLayer);

    // Blur (remove nan -> filter -> put back nan
    grid_map::inpainting::minValues(messageMap, elevationLayer, "i");
    grid_map::smoothing::boxBlur(messageMap, "i", elevationLayer, 3, 1);
    messageMap.get(elevationLayer) = (originalMap.array().isFinite()).select(messageMap.get(elevationLayer), originalMap);
  }

  auto& elevation = messageMap.get(elevationLayer);
  if (noiseLayer.size() != elevation.size()) {
    createNoise(elevation.rows(), elevation.cols());
  }

  elevation += noiseLayer;

  grid_map_msgs::GridMap messageMapOut;
  grid_map::GridMapRosConverter::toMessage(messageMap, messageMapOut);
  publisher.publish(messageMapOut);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "noise_node");
  ros::NodeHandle nodeHandle("~");

  if (!nodeHandle.getParam("frequency", frequency)) {
    ROS_ERROR("[ConvexPlaneExtractionROS::NoiseNode] Could not read parameter `frequency`.");
    return 1;
  }
  if (!nodeHandle.getParam("noiseGauss", noiseGauss)) {
    ROS_ERROR("[ConvexPlaneExtractionROS:NoiseNode] Could not read parameter `noiseGauss`.");
    return 1;
  }
  if (!nodeHandle.getParam("noiseUniform", noiseUniform)) {
    ROS_ERROR("[ConvexPlaneExtractionROS:NoiseNode] Could not read parameter `noiseUniform`.");
    return 1;
  }
  if (!nodeHandle.getParam("blur", blur)) {
    ROS_ERROR("[ConvexPlaneExtractionROS:NoiseNode] Could not read parameter `blur`.");
    return 1;
  }
  if (!nodeHandle.getParam("outlier_percentage", outlierPercentage)) {
    ROS_ERROR("[ConvexPlaneExtractionROS:NoiseNode] Could not read parameter `outlier_percentage`.");
    return 1;
  }
  if (!nodeHandle.getParam("elevation_topic_in", elevationMapTopicIn)) {
    ROS_ERROR("[ConvexPlaneExtractionROS:NoiseNode] Could not read parameter `elevation_topic_in`.");
    return 1;
  }
  if (!nodeHandle.getParam("elevation_topic_out", elevationMapTopicOut)) {
    ROS_ERROR("[ConvexPlaneExtractionROS:NoiseNode] Could not read parameter `elevation_topic_out`.");
    return 1;
  }
  if (!nodeHandle.getParam("height_layer", elevationLayer)) {
    ROS_ERROR("[ConvexPlaneExtractionROS:NoiseNode] Could not read parameter `height_layer`.");
    return 1;
  }

  publisher = nodeHandle.advertise<grid_map_msgs::GridMap>(elevationMapTopicOut, 1);
  auto elevationMapSubscriber_ = nodeHandle.subscribe<grid_map_msgs::GridMap>(elevationMapTopicIn, 1, callback);

  ros::Rate rate(frequency);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}