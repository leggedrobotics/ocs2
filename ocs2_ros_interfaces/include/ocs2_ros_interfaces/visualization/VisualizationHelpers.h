/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include "ocs2_ros_interfaces/visualization/VisualizationColors.h"

#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>

// ROS messages
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

namespace ocs2 {

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

visualization_msgs::Marker getArrowBetweenPointsMsg(const Eigen::Vector3d& start, const Eigen::Vector3d& end, Color color);

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

}  // namespace ocs2
