//
// Created by rgrandia on 11.06.20.
//

#include <ros/ros.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <opencv2/imgcodecs.hpp>

int count = 0;
double frequency;
std::string elevationMapTopic;
std::string elevationLayer;
std::string imageName;

void callback(const grid_map_msgs::GridMap::ConstPtr& message) {
  grid_map::GridMap messageMap;
  grid_map::GridMapRosConverter::fromMessage(*message, messageMap);

  const auto& data = messageMap[elevationLayer];
  float maxHeight = std::numeric_limits<float>::lowest();
  float minHeight = std::numeric_limits<float>::max();
  for (int i = 0; i < data.rows(); i++) {
    for (int j = 0; j < data.cols(); j++) {
      const auto value = data(i, j);
      if (!std::isnan(value)) {
        maxHeight = std::max(maxHeight, value);
        minHeight = std::min(minHeight, value);
      }
    }
  }

  cv::Mat image;
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(messageMap, elevationLayer, CV_8UC1, minHeight, maxHeight, image);

  int range = 100 * (maxHeight - minHeight);
  cv::imwrite(imageName + "_" + std::to_string(count++) + "_" + std::to_string(range) + "cm.png", image);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "save_elevation_map_to_image");
  ros::NodeHandle nodeHandle("~");

  if (!nodeHandle.getParam("frequency", frequency)) {
    ROS_ERROR("[ConvexPlaneExtractionROS] Could not read parameter `frequency`.");
    return 1;
  }
  if (!nodeHandle.getParam("elevation_topic", elevationMapTopic)) {
    ROS_ERROR("[ConvexPlaneExtractionROS] Could not read parameter `elevation_topic`.");
    return 1;
  }
  if (!nodeHandle.getParam("height_layer", elevationLayer)) {
    ROS_ERROR("[ConvexPlaneExtractionROS] Could not read parameter `height_layer`.");
    return 1;
  }
  if (!nodeHandle.getParam("imageName", imageName)) {
    ROS_ERROR("[ConvexPlaneExtractionROS] Could not read parameter `imageName`.");
    return 1;
  }

  auto elevationMapSubscriber_ = nodeHandle.subscribe<grid_map_msgs::GridMap>(elevationMapTopic, 1, callback);

  ros::Rate rate(frequency);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}