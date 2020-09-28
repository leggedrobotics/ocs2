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

int main(int argc, char* argv[]) {
  // This node publishes wheel angles to be zero

  // Initialize ros node
  ros::init(argc, argv, "anymal_mpc_test_terrain");
  ros::NodeHandle nodeHandle;

  auto regionPublisher_ =
      nodeHandle.advertise<convex_plane_decomposition_msgs::PlanarTerrain>("/convex_plane_decomposition_ros/planar_terrain", 1);
  auto boundaryPublisher_ = nodeHandle.advertise<jsk_recognition_msgs::PolygonArray>("boundaries", 1);
  auto insetPublisher_ = nodeHandle.advertise<jsk_recognition_msgs::PolygonArray>("insets", 1);

  double radius = 0.5;
  double inset = 0.05;
  CgalPolygonWithHoles2d hexagon(createRegularPolygon({0.0, 0.0}, radius, 6));
  CgalPolygonWithHoles2d hexagon_inset(createRegularPolygon({0.0, 0.0}, radius - inset, 6));

  PlanarRegion planarRegion;
  planarRegion.boundaryWithInset = BoundaryWithInset{hexagon, {hexagon_inset}};
  planarRegion.bbox2d = hexagon.outer_boundary().bbox();
  planarRegion.planeParameters = TerrainPlane();

  PlanarTerrain planarTerrain;
  planarTerrain.planarRegions = {planarRegion};
  planarTerrain.gridMap.setFrameId("world");

  ros::WallRate rate(1.0);
  while (ros::ok() && ros::master::check()) {
    regionPublisher_.publish(toMessage(planarTerrain));
    boundaryPublisher_.publish(convertBoundariesToRosPolygons(planarTerrain.planarRegions, planarTerrain.gridMap.getFrameId()));
    insetPublisher_.publish(convertInsetsToRosPolygons(planarTerrain.planarRegions, planarTerrain.gridMap.getFrameId()));

    rate.sleep();
  }

  return 0;
}
