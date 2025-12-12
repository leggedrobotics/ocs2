#include "convex_plane_decomposition_ros/RosVisualizations.h"

#include <algorithm>
#include <array>
#include <iterator>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace convex_plane_decomposition {

geometry_msgs::msg::PolygonStamped to3dRosPolygon(const CgalPolygon2d& polygon, const Eigen::Isometry3d& transformPlaneToWorld,
                                                  const std_msgs::msg::Header& header) {
  geometry_msgs::msg::PolygonStamped polygon3d;
  polygon3d.header = header;
  polygon3d.polygon.points.reserve(polygon.size());
  for (const auto& point : polygon) {
    geometry_msgs::msg::Point32 point_ros;
    const auto pointInWorld = positionInWorldFrameFromPosition2dInPlane(point, transformPlaneToWorld);
    point_ros.x = static_cast<float>(pointInWorld.x());
    point_ros.y = static_cast<float>(pointInWorld.y());
    point_ros.z = static_cast<float>(pointInWorld.z());
    polygon3d.polygon.points.push_back(point_ros);
  }
  return polygon3d;
}

std::vector<geometry_msgs::msg::PolygonStamped> to3dRosPolygon(const CgalPolygonWithHoles2d& polygonWithHoles,
                                                               const Eigen::Isometry3d& transformPlaneToWorld,
                                                               const std_msgs::msg::Header& header) {
  std::vector<geometry_msgs::msg::PolygonStamped> polygons;

  polygons.reserve(polygonWithHoles.number_of_holes() + 1);
  polygons.emplace_back(to3dRosPolygon(polygonWithHoles.outer_boundary(), transformPlaneToWorld, header));

  for (const auto& hole : polygonWithHoles.holes()) {
    polygons.emplace_back(to3dRosPolygon(hole, transformPlaneToWorld, header));
  }
  return polygons;
}

namespace {  // Helper functions for convertBoundariesToRosMarkers and convertInsetsToRosMarkers
std_msgs::msg::ColorRGBA getColor(int id, float alpha = 1.0) {
  constexpr int numColors = 7;
  using RGB = std::array<float, 3>;
  // clang-format off
  static const std::array<std::array<float, 3>, numColors> colorMap{
    RGB{0.0000F, 0.4470F, 0.7410F},
    RGB{0.8500F, 0.3250F, 0.0980F},
    RGB{0.9290F, 0.6940F, 0.1250F},
    RGB{0.4940F, 0.1840F, 0.5560F},
    RGB{0.4660F, 0.6740F, 0.1880F},
    RGB{0.6350F, 0.0780F, 0.1840F},
    RGB{0.2500F, 0.2500F, 0.2500F}
  };
  // clang-format on

  std_msgs::msg::ColorRGBA colorMsg;
  const auto& rgb = colorMap[id % numColors];
  colorMsg.r = rgb[0];
  colorMsg.g = rgb[1];
  colorMsg.b = rgb[2];
  colorMsg.a = alpha;
  return colorMsg;
}

visualization_msgs::msg::Marker to3dRosMarker(const CgalPolygon2d& polygon, const Eigen::Isometry3d& transformPlaneToWorld,
                                              const std_msgs::msg::Header& header, const std_msgs::msg::ColorRGBA& color, int id,
                                              double lineWidth) {
  visualization_msgs::msg::Marker line;
  line.id = id;
  line.header = header;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.scale.x = lineWidth;
  line.color = color;
  if (!polygon.is_empty()) {
    line.points.reserve(polygon.size() + 1);
    for (const auto& point : polygon) {
      const auto pointInWorld = positionInWorldFrameFromPosition2dInPlane(point, transformPlaneToWorld);
      geometry_msgs::msg::Point point_ros;
      point_ros.x = pointInWorld.x();
      point_ros.y = pointInWorld.y();
      point_ros.z = pointInWorld.z();
      line.points.push_back(point_ros);
    }
    // repeat the first point to close to polygon
    const auto pointInWorld = positionInWorldFrameFromPosition2dInPlane(polygon.vertex(0), transformPlaneToWorld);
    geometry_msgs::msg::Point point_ros;
    point_ros.x = pointInWorld.x();
    point_ros.y = pointInWorld.y();
    point_ros.z = pointInWorld.z();
    line.points.push_back(point_ros);
  }
  line.pose.orientation.w = 1.0;
  line.pose.orientation.x = 0.0;
  line.pose.orientation.y = 0.0;
  line.pose.orientation.z = 0.0;
  return line;
}

visualization_msgs::msg::MarkerArray to3dRosMarker(const CgalPolygonWithHoles2d& polygonWithHoles,
                                                   const Eigen::Isometry3d& transformPlaneToWorld, const std_msgs::msg::Header& header,
                                                   const std_msgs::msg::ColorRGBA& color, int id, double lineWidth) {
  visualization_msgs::msg::MarkerArray polygons;

  polygons.markers.reserve(polygonWithHoles.number_of_holes() + 1);
  polygons.markers.emplace_back(to3dRosMarker(polygonWithHoles.outer_boundary(), transformPlaneToWorld, header, color, id, lineWidth));
  ++id;

  for (const auto& hole : polygonWithHoles.holes()) {
    polygons.markers.emplace_back(to3dRosMarker(hole, transformPlaneToWorld, header, color, id, lineWidth));
    ++id;
  }
  return polygons;
}
}  // namespace

visualization_msgs::msg::MarkerArray convertBoundariesToRosMarkers(const std::vector<PlanarRegion>& planarRegions, const std::string& frameId,
                                                                   grid_map::Time time, double lineWidth) {
  std_msgs::msg::Header header;
  header.stamp = rclcpp::Time(static_cast<int64_t>(time));
  header.frame_id = frameId;

  visualization_msgs::msg::Marker deleteMarker;
  deleteMarker.action = visualization_msgs::msg::Marker::DELETEALL;

  visualization_msgs::msg::MarkerArray polygon_buffer;
  polygon_buffer.markers.reserve(planarRegions.size() + 1);  // lower bound
  polygon_buffer.markers.push_back(deleteMarker);
  int colorIdx = 0;
  for (const auto& planarRegion : planarRegions) {
    const auto color = getColor(colorIdx++);
    int label = polygon_buffer.markers.size();
    auto boundaries =
        to3dRosMarker(planarRegion.boundaryWithInset.boundary, planarRegion.transformPlaneToWorld, header, color, label, lineWidth);
    std::move(boundaries.markers.begin(), boundaries.markers.end(), std::back_inserter(polygon_buffer.markers));
  }

  return polygon_buffer;
}

visualization_msgs::msg::MarkerArray convertInsetsToRosMarkers(const std::vector<PlanarRegion>& planarRegions, const std::string& frameId,
                                                               grid_map::Time time, double lineWidth) {
  std_msgs::msg::Header header;
  header.stamp = rclcpp::Time(static_cast<int64_t>(time));
  header.frame_id = frameId;

  visualization_msgs::msg::Marker deleteMarker;
  deleteMarker.action = visualization_msgs::msg::Marker::DELETEALL;

  visualization_msgs::msg::MarkerArray polygon_buffer;
  polygon_buffer.markers.reserve(planarRegions.size() + 1);  // lower bound
  polygon_buffer.markers.push_back(deleteMarker);
  int colorIdx = 0;
  for (const auto& planarRegion : planarRegions) {
    const auto color = getColor(colorIdx++);
    for (const auto& inset : planarRegion.boundaryWithInset.insets) {
      int label = polygon_buffer.markers.size();
      auto insets = to3dRosMarker(inset, planarRegion.transformPlaneToWorld, header, color, label, lineWidth);
      std::move(insets.markers.begin(), insets.markers.end(), std::back_inserter(polygon_buffer.markers));
    }
  }
  return polygon_buffer;
}

}  // namespace convex_plane_decomposition
