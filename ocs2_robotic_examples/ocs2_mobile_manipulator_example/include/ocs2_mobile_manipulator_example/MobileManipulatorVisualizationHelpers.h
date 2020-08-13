#pragma once

#include <array>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

namespace mobile_manipulator {

Eigen::VectorXd getArmJointPositions(Eigen::VectorXd state);

Eigen::Vector3d getBasePosition(Eigen::VectorXd state);

Eigen::Quaterniond getBaseOrientation(Eigen::VectorXd state);

/* Helpers by Ruben Grandia from ocs2_quadruped_interface/QuadrupedVisualizationHelpers.h */

template <typename It>
void assignHeader(It firstIt, It lastIt, const std_msgs::Header& header) {
  for (; firstIt != lastIt; ++firstIt) {
    firstIt->header = header;
  }
}

template <typename It>
void assignIncreasingId(It firstIt, It lastIt, int startId = 0) {
  for (; firstIt != lastIt; ++firstIt) {
    firstIt->id = startId++;
  }
}

geometry_msgs::Vector3 getVectorMsg(const Eigen::Vector3d& vec);

geometry_msgs::Point getPointMsg(const Eigen::Vector3d& point);

geometry_msgs::Quaternion getOrientationMsg(const Eigen::Quaterniond& orientation);

std_msgs::Header getHeaderMsg(const std::string& frame_id, const ros::Time& timeStamp);

visualization_msgs::Marker getLineMsg(std::vector<geometry_msgs::Point>&& points, std::array<double, 3> color, double lineWidth);

std_msgs::ColorRGBA getColor(std::array<double, 3> rgb, double alpha = 1.0);

}  // namespace mobile_manipulator
