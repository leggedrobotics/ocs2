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

#include "ocs2_ros_interfaces/visualization/VisualizationHelpers.h"

namespace ocs2 {

std_msgs::msg::ColorRGBA getColor(Color color, double alpha) {
  const auto rgb = getRGB(color);
  std_msgs::msg::ColorRGBA colorMsg;
  colorMsg.r = rgb[0];
  colorMsg.g = rgb[1];
  colorMsg.b = rgb[2];
  colorMsg.a = alpha;
  return colorMsg;
}

void setVisible(visualization_msgs::msg::Marker& marker) {
  marker.color.a = 1.0;
}

void setInvisible(visualization_msgs::msg::Marker& marker) {
  marker.color.a = 0.001;  // Rviz creates a warning when a is set to 0
}

std_msgs::msg::Header getHeaderMsg(const std::string& frame_id,  const rclcpp::Time& timeStamp) {
  std_msgs::msg::Header header;
  header.frame_id = frame_id;
  header.stamp = timeStamp;
  return header;
}

visualization_msgs::msg::Marker getLineMsg(std::vector<geometry_msgs::msg::Point>&& points, Color color, double lineWidth) {
  visualization_msgs::msg::Marker line;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.scale.x = lineWidth;
  line.color = getColor(color);
  line.points = std::move(points);
  line.pose.orientation = getOrientationMsg({1., 0., 0., 0.});
  return line;
}

geometry_msgs::msg::Point getPointMsg(const Eigen::Vector3d& point) {
  geometry_msgs::msg::Point pointMsg;
  pointMsg.x = point.x();
  pointMsg.y = point.y();
  pointMsg.z = point.z();
  return pointMsg;
}

geometry_msgs::msg::Vector3 getVectorMsg(const Eigen::Vector3d& vec) {
  geometry_msgs::msg::Vector3 vecMsg;
  vecMsg.x = vec.x();
  vecMsg.y = vec.y();
  vecMsg.z = vec.z();
  return vecMsg;
}

geometry_msgs::msg::Quaternion getOrientationMsg(const Eigen::Quaterniond& orientation) {
  geometry_msgs::msg::Quaternion orientationMsg;
  orientationMsg.x = orientation.x();
  orientationMsg.y = orientation.y();
  orientationMsg.z = orientation.z();
  orientationMsg.w = orientation.w();
  return orientationMsg;
}

visualization_msgs::msg::Marker getSphereMsg(const Eigen::Vector3d& point, Color color, double diameter) {
  visualization_msgs::msg::Marker sphere;
  sphere.type = visualization_msgs::msg::Marker::SPHERE;
  sphere.pose.position = getPointMsg(point);
  sphere.pose.orientation = getOrientationMsg({1., 0., 0., 0.});
  sphere.scale.x = diameter;
  sphere.scale.y = diameter;
  sphere.scale.z = diameter;
  sphere.color = getColor(color);
  return sphere;
}

visualization_msgs::msg::Marker getPlaneMsg(const Eigen::Vector3d& point, const Eigen::Quaterniond& orientation, Color color, double width,
                                       double length, double thickness) {
  visualization_msgs::msg::Marker plane;
  plane.type = visualization_msgs::msg::Marker::CUBE;
  plane.pose.position = getPointMsg(point);
  plane.pose.orientation = getOrientationMsg(orientation);
  plane.scale.x = length;
  plane.scale.y = width;
  plane.scale.z = thickness;
  plane.color = getColor(color);
  return plane;
}

visualization_msgs::msg::Marker getArrowToPointMsg(const Eigen::Vector3d& vec, const Eigen::Vector3d& point, Color color) {
  return getArrowBetweenPointsMsg(point - vec, point, color);
}

visualization_msgs::msg::Marker getArrowAtPointMsg(const Eigen::Vector3d& vec, const Eigen::Vector3d& point, Color color) {
  return getArrowBetweenPointsMsg(point, point + vec, color);
}

visualization_msgs::msg::Marker getArrowBetweenPointsMsg(const Eigen::Vector3d& start, const Eigen::Vector3d& end, Color color) {
  visualization_msgs::msg::Marker arrow;
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.scale.x = 0.01;  // shaft diameter
  arrow.scale.y = 0.02;  // arrow-head diameter
  arrow.scale.z = 0.06;  // arrow-head length
  arrow.points.reserve(2);
  arrow.points.emplace_back(getPointMsg(start));  // start point
  arrow.points.emplace_back(getPointMsg(end));    // end point
  arrow.color = getColor(color);
  arrow.pose.orientation = getOrientationMsg({1., 0., 0., 0.});
  return arrow;
}

visualization_msgs::msg::Marker getFootMarker(const Eigen::Vector3d& position, bool contactFlag, Color color, double diameter,
                                         double liftedAlpha) {
  auto footMarker = getSphereMsg(position, color, diameter);
  if (!contactFlag) {
    footMarker.color.a = liftedAlpha;
  }
  footMarker.ns = "EE Positions";
  return footMarker;
}

visualization_msgs::msg::Marker getForceMarker(const Eigen::Vector3d& force, const Eigen::Vector3d& position, bool contactFlag, Color color,
                                          double forceScale) {
  auto forceMarker = getArrowToPointMsg(force / forceScale, position, color);
  forceMarker.ns = "EE Forces";
  if (!contactFlag) {
    setInvisible(forceMarker);
  }
  return forceMarker;
}

}  // namespace ocs2
