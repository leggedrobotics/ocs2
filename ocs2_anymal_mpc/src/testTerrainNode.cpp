//
// Created by rgrandia on 28.09.20.
//

#include <ros/init.h>
#include <ros/master.h>
#include <ros/node_handle.h>

#include <jsk_recognition_msgs/PolygonArray.h>

#include <convex_plane_decomposition/ConvexRegionGrowing.h>
#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition_msgs/PlanarTerrain.h>
#include <convex_plane_decomposition_ros/MessageConversion.h>
#include <convex_plane_decomposition_ros/RosVisualizations.h>

using namespace convex_plane_decomposition;

PlanarRegion createPolygonRegion(const CgalPoint2d& center, double radius, int numberOfVertices) {
  const double inset = 0.05;
  radius = std::max(radius, 1.001 * inset);
  // Center around {0, 0}. Shift of the polygon happens through plane parameters.
  CgalPolygonWithHoles2d boundary(createRegularPolygon({0.0, 0.0}, radius, numberOfVertices));
  CgalPolygonWithHoles2d boundaryInset(createRegularPolygon({0.0, 0.0}, radius - inset, numberOfVertices));

  PlanarRegion planarRegion;
  planarRegion.boundaryWithInset = BoundaryWithInset{boundary, {boundaryInset}};
  planarRegion.bbox2d = boundary.outer_boundary().bbox();
  planarRegion.planeParameters = TerrainPlane();
  planarRegion.planeParameters.positionInWorld.x() = center.x();
  planarRegion.planeParameters.positionInWorld.y() = center.y();

  std::cout << planarRegion.planeParameters.orientationWorldToTerrain << std::endl << std::endl;

  return planarRegion;
}

int main(int argc, char* argv[]) {
  // Initialize ros node
  ros::init(argc, argv, "anymal_mpc_test_terrain");
  ros::NodeHandle nodeHandle;

  auto regionPublisher_ =
      nodeHandle.advertise<convex_plane_decomposition_msgs::PlanarTerrain>("/convex_plane_decomposition_ros/planar_terrain", 1);
  auto boundaryPublisher_ = nodeHandle.advertise<jsk_recognition_msgs::PolygonArray>("/convex_plane_decomposition_ros/boundaries", 1);
  auto insetPublisher_ = nodeHandle.advertise<jsk_recognition_msgs::PolygonArray>("/convex_plane_decomposition_ros/insets", 1);

  const double spacingRadius = 0.5;
  const double hexRadius = 0.4;

  const double hexagon_dy = spacingRadius * std::cos(M_PI / 6.0);
  const double hexagon_dx = spacingRadius * (1.0 + std::sin(M_PI / 6.0));

  PlanarTerrain planarTerrain;
  planarTerrain.planarRegions.emplace_back(createPolygonRegion({hexagon_dx, 0.0}, hexRadius, 6));
  planarTerrain.planarRegions.emplace_back(createPolygonRegion({-hexagon_dx, 0.0}, hexRadius, 6));
  planarTerrain.planarRegions.emplace_back(createPolygonRegion({0.0, hexagon_dy}, hexRadius, 6));
  planarTerrain.planarRegions.emplace_back(createPolygonRegion({0.0, -hexagon_dy}, hexRadius, 6));
  planarTerrain.gridMap.setFrameId("world");

  ros::WallRate rate(1.0);
  while (ros::ok() && ros::master::check()) {
    regionPublisher_.publish(toMessage(planarTerrain));
    const auto time = ros::Time::now();
    boundaryPublisher_.publish(
        convertBoundariesToRosPolygons(planarTerrain.planarRegions, planarTerrain.gridMap.getFrameId(), time.toNSec()));
    insetPublisher_.publish(convertInsetsToRosPolygons(planarTerrain.planarRegions, planarTerrain.gridMap.getFrameId(), time.toNSec()));

    rate.sleep();
  }

  return 0;
}
