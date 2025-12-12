#include "convex_plane_decomposition_ros/ConvexPlaneDecompositionRos.h"

#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "convex_plane_decomposition_ros");
  ros::NodeHandle nodeHandle("~");

  double frequency;
  if (!nodeHandle.getParam("frequency", frequency)) {
    ROS_ERROR("[ConvexPlaneDecompositionNode] Could not read parameter `frequency`.");
    return 1;
  }

  convex_plane_decomposition::ConvexPlaneExtractionROS convex_plane_decomposition_ros(nodeHandle);

  ros::Rate rate(frequency);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
