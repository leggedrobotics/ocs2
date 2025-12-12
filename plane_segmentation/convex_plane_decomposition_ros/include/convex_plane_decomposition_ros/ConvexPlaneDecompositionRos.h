#pragma once

#include <memory>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

#include <grid_map_msgs/GridMap.h>

#include <convex_plane_decomposition/Timer.h>

namespace convex_plane_decomposition {

// Forward declaration of the pipeline
class PlaneDecompositionPipeline;

class ConvexPlaneExtractionROS {
 public:
  ConvexPlaneExtractionROS(ros::NodeHandle& nodeHandle);

  ~ConvexPlaneExtractionROS();

 private:
  bool loadParameters(const ros::NodeHandle& nodeHandle);

  /**
   * Callback method for the incoming grid map message.
   * @param message the incoming message.
   */
  void callback(const grid_map_msgs::GridMap& message);

  Eigen::Isometry3d getTransformToTargetFrame(const std::string& sourceFrame, const ros::Time& time);

  // Parameters
  std::string elevationMapTopic_;
  std::string elevationLayer_;
  std::string targetFrameId_;
  double subMapWidth_;
  double subMapLength_;
  bool publishToController_;

  // ROS communication
  ros::Subscriber elevationMapSubscriber_;
  ros::Publisher filteredmapPublisher_;
  ros::Publisher boundaryPublisher_;
  ros::Publisher insetPublisher_;
  ros::Publisher regionPublisher_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  // Pipeline
  std::unique_ptr<PlaneDecompositionPipeline> planeDecompositionPipeline_;

  // Timing
  Timer callbackTimer_;
};

}  // namespace convex_plane_decomposition
