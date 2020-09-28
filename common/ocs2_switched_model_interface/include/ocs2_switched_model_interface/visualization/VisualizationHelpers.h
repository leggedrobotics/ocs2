//
// Created by rgrandia on 20.02.20.
//

#pragma once

#include "Colors.h"

#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>

// ROS messages
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

namespace switched_model {

std_msgs::ColorRGBA getColor(Color color, double alpha = 1.0);

void setVisible(visualization_msgs::Marker& marker);

void setInvisible(visualization_msgs::Marker& marker);

std_msgs::Header getHeaderMsg(const std::string& frame_id, const ros::Time& timeStamp);

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

visualization_msgs::Marker getLineMsg(std::vector<geometry_msgs::Point>&& points, Color color, double lineWidth);

geometry_msgs::Point getPointMsg(const Eigen::Vector3d& point);

geometry_msgs::Vector3 getVectorMsg(const Eigen::Vector3d& vec);

geometry_msgs::Quaternion getOrientationMsg(const Eigen::Quaterniond& orientation);

visualization_msgs::Marker getSphereMsg(const Eigen::Vector3d& point, Color color, double diameter);

visualization_msgs::Marker getPlaneMsg(const Eigen::Vector3d& point, const Eigen::Quaterniond& orientation, Color color, double width,
                                       double length, double thickness);

visualization_msgs::Marker getArrowToPointMsg(const Eigen::Vector3d& vec, const Eigen::Vector3d& point, Color color);

visualization_msgs::Marker getArrowAtPointMsg(const Eigen::Vector3d& vec, const Eigen::Vector3d& point, Color color);

visualization_msgs::Marker getFootMarker(const Eigen::Vector3d& position, bool contactFlag, Color color, double diameter,
                                         double liftedAlpha);

visualization_msgs::Marker getForceMarker(const Eigen::Vector3d& force, const Eigen::Vector3d& position, bool contactFlag, Color color,
                                          double forceScale);

template <typename ForceIt, typename PositionIt, typename ContactIt>
visualization_msgs::Marker getCenterOfPressureMarker(ForceIt firstForce, ForceIt lastForce, PositionIt positionIt, ContactIt contactIt,
                                                     Color color, double diameter) {
  // Compute center of pressure
  Eigen::Vector3d centerOfPressure = Eigen::Vector3d::Zero();
  double sum_z = 0.0;
  int numContacts = 0;
  for (; firstForce != lastForce; ++firstForce, ++positionIt, ++contactIt) {
    sum_z += firstForce->z();
    centerOfPressure += firstForce->z() * (*positionIt);
    numContacts += (*contactIt) ? 1 : 0;
  }
  if (sum_z > 0) {
    centerOfPressure /= sum_z;
  }

  // Construct marker
  visualization_msgs::Marker copMarker = getSphereMsg(centerOfPressure, color, diameter);
  if (numContacts == 0) {
    setInvisible(copMarker);
  }
  copMarker.ns = "Center of Pressure";
  copMarker.pose.orientation = getOrientationMsg({1., 0., 0., 0.});
  return copMarker;
}

template <typename PositionIt, typename ContactIt>
visualization_msgs::Marker getSupportPolygonMarker(PositionIt firstPos, PositionIt lastPos, ContactIt contactIt, Color color,
                                                   double lineWidth) {
  visualization_msgs::Marker lineList;
  lineList.type = visualization_msgs::Marker::LINE_LIST;
  auto numElements = std::distance(firstPos, lastPos);
  lineList.points.reserve(numElements * (numElements - 1) / 2);  // Upper bound on the number of lines

  // Loop over all positions
  for (; firstPos != lastPos; ++firstPos, ++contactIt) {
    // For each position, loop over all future positions in the container
    auto nextPos = std::next(firstPos);
    auto nextContact = std::next(contactIt);
    for (; nextPos != lastPos; ++nextPos, ++nextContact) {
      if (*contactIt && *nextContact) {
        // When positions are both marked as in contact, draw a line between the two points
        lineList.points.push_back(getPointMsg(*firstPos));
        lineList.points.push_back(getPointMsg(*nextPos));
      }
    }
  }
  lineList.scale.x = lineWidth;
  lineList.color = getColor(color);
  lineList.ns = "Support Polygon";
  lineList.pose.orientation = getOrientationMsg({1., 0., 0., 0.});
  return lineList;
}

template <typename F>
double timedExecutionInSeconds(F func) {
  auto start = std::chrono::steady_clock::now();
  func();
  auto finish = std::chrono::steady_clock::now();
  return std::chrono::duration_cast<std::chrono::duration<double>>(finish - start).count();
}

}  // namespace switched_model
