#pragma once

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <convex_plane_decomposition/PlanarRegion.h>

namespace convex_plane_decomposition {

geometry_msgs::msg::PolygonStamped to3dRosPolygon(const CgalPolygon2d& polygon, const Eigen::Isometry3d& transformPlaneToWorld,
                                                  const std_msgs::msg::Header& header);

std::vector<geometry_msgs::msg::PolygonStamped> to3dRosPolygon(const CgalPolygonWithHoles2d& polygonWithHoles,
                                                               const Eigen::Isometry3d& transformPlaneToWorld,
                                                               const std_msgs::msg::Header& header);

visualization_msgs::msg::MarkerArray convertBoundariesToRosMarkers(const std::vector<PlanarRegion>& planarRegions,
                                                                   const std::string& frameId, grid_map::Time time,
                                                                   double lineWidth);

visualization_msgs::msg::MarkerArray convertInsetsToRosMarkers(const std::vector<PlanarRegion>& planarRegions,
                                                               const std::string& frameId, grid_map::Time time,
                                                               double lineWidth);

}  // namespace convex_plane_decomposition
