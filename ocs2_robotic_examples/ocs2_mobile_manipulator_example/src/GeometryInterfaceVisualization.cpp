/*
 * GeometryInterfaceVisualization.cpp
 *
 *  Created on: 4 Sep 2020
 *      Author: perry
 */

#include <pinocchio/fwd.hpp>

#include <ocs2_mobile_manipulator_example/GeometryInterfaceVisualization.h>

#include <ocs2_mobile_manipulator_example/MobileManipulatorVisualizationHelpers.h>

namespace ocs2 {

GeometryInterfaceVisualization::GeometryInterfaceVisualization(const PinocchioGeometryInterface& geometryInterface, ros::NodeHandle& nh,
                                                               const std::string& pinocchioWorldFrame)
    : geometryInterface_(geometryInterface),
      markerPublisher_(nh.advertise<visualization_msgs::MarkerArray>("distance_markers", 1, true)),
      pinocchioWorldFrame_(pinocchioWorldFrame) {}

void GeometryInterfaceVisualization::publishDistances(const ocs2::state_vector_t& q) {
  std::vector<hpp::fcl::DistanceResult> results = geometryInterface_.computeDistances(q);

  visualization_msgs::MarkerArray markerArray;

  constexpr size_t numMarkersPerResult = 5;

  visualization_msgs::Marker markerTemplate;
  markerTemplate.color = mobile_manipulator::getColor({0, 1, 0}, 1);
  markerTemplate.header.frame_id = pinocchioWorldFrame_;
  markerTemplate.header.stamp = ros::Time::now();
  markerTemplate.pose.orientation = mobile_manipulator::getOrientationMsg({1, 0, 0, 0});
  markerArray.markers.resize(results.size() * numMarkersPerResult, markerTemplate);

  for (size_t i = 0; i < results.size(); ++i) {
    // I apologize for the magic numbers, it's mostly just visualization numbers(so 0.02 scale corresponds rougly to 0.02 cm)
    // The actual distance line, also denoting direction of the distance
    markerArray.markers[numMarkersPerResult * i].type = visualization_msgs::Marker::ARROW;
    markerArray.markers[numMarkersPerResult * i].points.push_back(mobile_manipulator::getPointMsg(results[i].nearest_points[0]));
    markerArray.markers[numMarkersPerResult * i].points.push_back(mobile_manipulator::getPointMsg(results[i].nearest_points[1]));
    markerArray.markers[numMarkersPerResult * i].id = numMarkersPerResult * i;
    markerArray.markers[numMarkersPerResult * i].scale.x = 0.01;
    markerArray.markers[numMarkersPerResult * i].scale.y = 0.02;
    markerArray.markers[numMarkersPerResult * i].scale.z = 0.04;
    // Dots at the end of the arrow, denoting the close locations on the body
    markerArray.markers[numMarkersPerResult * i + 1].type = visualization_msgs::Marker::SPHERE_LIST;
    markerArray.markers[numMarkersPerResult * i + 1].points.push_back(mobile_manipulator::getPointMsg(results[i].nearest_points[0]));
    markerArray.markers[numMarkersPerResult * i + 1].points.push_back(mobile_manipulator::getPointMsg(results[i].nearest_points[1]));
    markerArray.markers[numMarkersPerResult * i + 1].scale.x = 0.02;
    markerArray.markers[numMarkersPerResult * i + 1].scale.y = 0.02;
    markerArray.markers[numMarkersPerResult * i + 1].scale.z = 0.02;
    markerArray.markers[numMarkersPerResult * i + 1].id = numMarkersPerResult * i + 1;
    // Text denoting the object number in the geometry model, raised above the spheres
    markerArray.markers[numMarkersPerResult * i + 2].id = numMarkersPerResult * i + 2;
    markerArray.markers[numMarkersPerResult * i + 2].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    markerArray.markers[numMarkersPerResult * i + 2].scale.z = 0.02;
    markerArray.markers[numMarkersPerResult * i + 2].pose.position = mobile_manipulator::getPointMsg(results[i].nearest_points[0]);
    markerArray.markers[numMarkersPerResult * i + 2].pose.position.z += 0.015;
    markerArray.markers[numMarkersPerResult * i + 2].text =
        "obj " + std::to_string(geometryInterface_.getGeometryModel().collisionPairs[i].first);
    markerArray.markers[numMarkersPerResult * i + 3].id = numMarkersPerResult * i + 3;
    markerArray.markers[numMarkersPerResult * i + 3].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    markerArray.markers[numMarkersPerResult * i + 3].pose.position = mobile_manipulator::getPointMsg(results[i].nearest_points[1]);
    markerArray.markers[numMarkersPerResult * i + 3].pose.position.z += 0.015;
    markerArray.markers[numMarkersPerResult * i + 3].text =
        "obj " + std::to_string(geometryInterface_.getGeometryModel().collisionPairs[i].second);
    markerArray.markers[numMarkersPerResult * i + 3].scale.z = 0.02;
    // Text above the arrow, denoting the distance
    markerArray.markers[numMarkersPerResult * i + 4].id = numMarkersPerResult * i + 4;
    markerArray.markers[numMarkersPerResult * i + 4].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    markerArray.markers[numMarkersPerResult * i + 4].pose.position =
        mobile_manipulator::getPointMsg((results[i].nearest_points[0] + results[i].nearest_points[1]) / 2.0);
    markerArray.markers[numMarkersPerResult * i + 4].pose.position.z += 0.015;
    markerArray.markers[numMarkersPerResult * i + 4].text =
        "dist " + std::to_string(geometryInterface_.getGeometryModel().collisionPairs[i].first) + " - " +
        std::to_string(geometryInterface_.getGeometryModel().collisionPairs[i].second) + ": " + std::to_string(results[i].min_distance);
    markerArray.markers[numMarkersPerResult * i + 4].scale.z = 0.02;
  }

  markerPublisher_.publish(markerArray);
}

}  // namespace ocs2
