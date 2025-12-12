//
// Created by rgrandia on 14.06.20.
//

#pragma once

#include <geometry_msgs/msg/pose.hpp>

#include <convex_plane_decomposition_msgs/msg/bounding_box2d.hpp>
#include <convex_plane_decomposition_msgs/msg/planar_region.hpp>
#include <convex_plane_decomposition_msgs/msg/planar_terrain.hpp>
#include <convex_plane_decomposition_msgs/msg/point2d.hpp>
#include <convex_plane_decomposition_msgs/msg/polygon2d.hpp>
#include <convex_plane_decomposition_msgs/msg/polygon_with_holes2d.hpp>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition/PolygonTypes.h>

namespace convex_plane_decomposition {

CgalBbox2d fromMessage(const convex_plane_decomposition_msgs::msg::BoundingBox2d& msg);
convex_plane_decomposition_msgs::msg::BoundingBox2d toMessage(const CgalBbox2d& bbox2d);

PlanarRegion fromMessage(const convex_plane_decomposition_msgs::msg::PlanarRegion& msg);
convex_plane_decomposition_msgs::msg::PlanarRegion toMessage(const PlanarRegion& planarRegion);

PlanarTerrain fromMessage(const convex_plane_decomposition_msgs::msg::PlanarTerrain& msg);
convex_plane_decomposition_msgs::msg::PlanarTerrain toMessage(const PlanarTerrain& planarTerrain);

Eigen::Isometry3d fromMessage(const geometry_msgs::msg::Pose& msg);
geometry_msgs::msg::Pose toMessage(const Eigen::Isometry3d& transform);

CgalPoint2d fromMessage(const convex_plane_decomposition_msgs::msg::Point2d& msg);
convex_plane_decomposition_msgs::msg::Point2d toMessage(const CgalPoint2d& point2d);

CgalPolygon2d fromMessage(const convex_plane_decomposition_msgs::msg::Polygon2d& msg);
convex_plane_decomposition_msgs::msg::Polygon2d toMessage(const CgalPolygon2d& polygon2d);

CgalPolygonWithHoles2d fromMessage(const convex_plane_decomposition_msgs::msg::PolygonWithHoles2d& msg);
convex_plane_decomposition_msgs::msg::PolygonWithHoles2d toMessage(const CgalPolygonWithHoles2d& polygonWithHoles2d);

}  // namespace convex_plane_decomposition
