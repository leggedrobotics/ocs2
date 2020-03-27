//
// Created by rgrandia on 20.02.20.
//

#include "ocs2_quadruped_interface/QuadrupedVisualizationHelpers.h"

namespace switched_model {

std_msgs::ColorRGBA getColor(Color color, double alpha) {
  const auto rgb = getRGB(color);
  std_msgs::ColorRGBA colorMsg;
  colorMsg.r = rgb[0];
  colorMsg.g = rgb[1];
  colorMsg.b = rgb[2];
  colorMsg.a = alpha;
  return colorMsg;
}

void setVisible(visualization_msgs::Marker& marker) {
  marker.color.a = 1.0;
}

void setInvisible(visualization_msgs::Marker& marker) {
  marker.color.a = 0.001;  // Rviz creates a warning when a is set to 0
}

std_msgs::Header getHeaderMsg(const std::string& frame_id, const ros::Time& timeStamp) {
  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = timeStamp;
  return header;
}

visualization_msgs::Marker getLineMsg(std::vector<geometry_msgs::Point>&& points, Color color, double lineWidth) {
  visualization_msgs::Marker line;
  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.scale.x = lineWidth;
  line.color = getColor(color);
  line.points = std::move(points);
  line.pose.orientation = getOrientationMsg({1., 0., 0., 0.});
  return line;
}

geometry_msgs::Point getPointMsg(const Eigen::Vector3d& point) {
  geometry_msgs::Point pointMsg;
  pointMsg.x = point.x();
  pointMsg.y = point.y();
  pointMsg.z = point.z();
  return pointMsg;
}

geometry_msgs::Vector3 getVectorMsg(const Eigen::Vector3d& vec) {
  geometry_msgs::Vector3 vecMsg;
  vecMsg.x = vec.x();
  vecMsg.y = vec.y();
  vecMsg.z = vec.z();
  return vecMsg;
}

geometry_msgs::Quaternion getOrientationMsg(const Eigen::Quaterniond& orientation) {
  geometry_msgs::Quaternion orientationMsg;
  orientationMsg.x = orientation.x();
  orientationMsg.y = orientation.y();
  orientationMsg.z = orientation.z();
  orientationMsg.w = orientation.w();
  return orientationMsg;
}

visualization_msgs::Marker getSphereMsg(const Eigen::Vector3d& point, Color color, double diameter) {
  visualization_msgs::Marker sphere;
  sphere.type = visualization_msgs::Marker::SPHERE;
  sphere.pose.position = getPointMsg(point);
  sphere.pose.orientation = getOrientationMsg({1., 0., 0., 0.});
  sphere.scale.x = diameter;
  sphere.scale.y = diameter;
  sphere.scale.z = diameter;
  sphere.color = getColor(color);
  return sphere;
}

visualization_msgs::Marker getArrowToPointMsg(const Eigen::Vector3d& vec, const Eigen::Vector3d& point, Color color) {
  visualization_msgs::Marker arrow;
  arrow.type = visualization_msgs::Marker::ARROW;
  arrow.scale.x = 0.01;                                 // shaft diameter
  arrow.scale.y = 0.02;                                 // arrow-head diameter
  arrow.scale.z = 0.06;                                 // arrow-head length
  arrow.points.emplace_back(getPointMsg(point - vec));  // start point
  arrow.points.emplace_back(getPointMsg(point));        // end point
  arrow.color = getColor(color);
  arrow.pose.orientation = getOrientationMsg({1., 0., 0., 0.});
  return arrow;
}

visualization_msgs::Marker getFootMarker(const Eigen::Vector3d& position, bool contactFlag, Color color, double diameter,
                                         double liftedAlpha) {
  auto footMarker = getSphereMsg(position, color, diameter);
  if (!contactFlag) {
    footMarker.color.a = liftedAlpha;
  }
  footMarker.ns = "EE Positions";
  return footMarker;
}

visualization_msgs::Marker getForceMarker(const Eigen::Vector3d& force, const Eigen::Vector3d& position, bool contactFlag, Color color,
                                          double forceScale) {
  auto forceMarker = getArrowToPointMsg(force / forceScale, position, color);
  forceMarker.ns = "EE Forces";
  if (!contactFlag) {
    setInvisible(forceMarker);
  }
  return forceMarker;
}

}  // namespace switched_model
