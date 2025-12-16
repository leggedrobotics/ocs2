/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include "ocs2_self_collision_visualization/GeometryInterfaceVisualization.h"

#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/geometry.hpp>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GeometryInterfaceVisualization::GeometryInterfaceVisualization(
    PinocchioInterface pinocchioInterface,
    PinocchioGeometryInterface geometryInterface,
    std::string pinocchioWorldFrame)
    : Node("GeometryInterfaceVisualization"),
      pinocchioInterface_(std::move(pinocchioInterface)),
      geometryInterface_(std::move(geometryInterface)),
      markerPublisher_(
          this->create_publisher<visualization_msgs::msg::MarkerArray>(
              "distance_markers", 1)),
      pinocchioWorldFrame_(std::move(pinocchioWorldFrame)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GeometryInterfaceVisualization::publishDistances(const ocs2::vector_t& q) {
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  pinocchio::forwardKinematics(model, data, q);
  const auto results = geometryInterface_.computeDistances(pinocchioInterface_);

  visualization_msgs::msg::MarkerArray markerArray;

  constexpr size_t numMarkersPerResult = 5;

  visualization_msgs::msg::Marker markerTemplate;
  markerTemplate.color = ros_msg_helpers::getColor({0, 1, 0}, 1);
  markerTemplate.header.frame_id = pinocchioWorldFrame_;
  markerTemplate.header.stamp = rclcpp::Clock().now();
  markerTemplate.pose.orientation =
      ros_msg_helpers::getOrientationMsg({1, 0, 0, 0});
  markerArray.markers.resize(results.size() * numMarkersPerResult,
                             markerTemplate);

  for (size_t i = 0; i < results.size(); ++i) {
    // I apologize for the magic numbers, it's mostly just visualization
    // numbers(so 0.02 scale corresponds rougly to 0.02 cm)

    for (size_t j = 0; j < numMarkersPerResult; ++j) {
      markerArray.markers[numMarkersPerResult * i + j].ns =
          std::to_string(
              geometryInterface_.getGeometryModel().collisionPairs[i].first) +
          " - " +
          std::to_string(
              geometryInterface_.getGeometryModel().collisionPairs[i].second);
    }

    // The actual distance line, also denoting direction of the distance
    markerArray.markers[numMarkersPerResult * i].type =
        visualization_msgs::msg::Marker::ARROW;
    markerArray.markers[numMarkersPerResult * i].points.push_back(
        ros_msg_helpers::getPointMsg(results[i].nearest_points[0]));
    markerArray.markers[numMarkersPerResult * i].points.push_back(
        ros_msg_helpers::getPointMsg(results[i].nearest_points[1]));
    markerArray.markers[numMarkersPerResult * i].id = numMarkersPerResult * i;
    markerArray.markers[numMarkersPerResult * i].scale.x = 0.01;
    markerArray.markers[numMarkersPerResult * i].scale.y = 0.02;
    markerArray.markers[numMarkersPerResult * i].scale.z = 0.04;
    // Dots at the end of the arrow, denoting the close locations on the body
    markerArray.markers[numMarkersPerResult * i + 1].type =
        visualization_msgs::msg::Marker::SPHERE_LIST;
    markerArray.markers[numMarkersPerResult * i + 1].points.push_back(
        ros_msg_helpers::getPointMsg(results[i].nearest_points[0]));
    markerArray.markers[numMarkersPerResult * i + 1].points.push_back(
        ros_msg_helpers::getPointMsg(results[i].nearest_points[1]));
    markerArray.markers[numMarkersPerResult * i + 1].scale.x = 0.02;
    markerArray.markers[numMarkersPerResult * i + 1].scale.y = 0.02;
    markerArray.markers[numMarkersPerResult * i + 1].scale.z = 0.02;
    markerArray.markers[numMarkersPerResult * i + 1].id =
        numMarkersPerResult * i + 1;
    // Text denoting the object number in the geometry model, raised above the
    // spheres
    markerArray.markers[numMarkersPerResult * i + 2].id =
        numMarkersPerResult * i + 2;
    markerArray.markers[numMarkersPerResult * i + 2].type =
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    markerArray.markers[numMarkersPerResult * i + 2].scale.z = 0.02;
    markerArray.markers[numMarkersPerResult * i + 2].pose.position =
        ros_msg_helpers::getPointMsg(results[i].nearest_points[0]);
    markerArray.markers[numMarkersPerResult * i + 2].pose.position.z += 0.015;
    markerArray.markers[numMarkersPerResult * i + 2].text =
        "obj " +
        std::to_string(
            geometryInterface_.getGeometryModel().collisionPairs[i].first);
    markerArray.markers[numMarkersPerResult * i + 3].id =
        numMarkersPerResult * i + 3;
    markerArray.markers[numMarkersPerResult * i + 3].type =
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    markerArray.markers[numMarkersPerResult * i + 3].pose.position =
        ros_msg_helpers::getPointMsg(results[i].nearest_points[1]);
    markerArray.markers[numMarkersPerResult * i + 3].pose.position.z += 0.015;
    markerArray.markers[numMarkersPerResult * i + 3].text =
        "obj " +
        std::to_string(
            geometryInterface_.getGeometryModel().collisionPairs[i].second);
    markerArray.markers[numMarkersPerResult * i + 3].scale.z = 0.02;
    // Text above the arrow, denoting the distance
    markerArray.markers[numMarkersPerResult * i + 4].id =
        numMarkersPerResult * i + 4;
    markerArray.markers[numMarkersPerResult * i + 4].type =
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    markerArray.markers[numMarkersPerResult * i + 4].pose.position =
        ros_msg_helpers::getPointMsg(
            (results[i].nearest_points[0] + results[i].nearest_points[1]) /
            2.0);
    markerArray.markers[numMarkersPerResult * i + 4].pose.position.z += 0.015;
    markerArray.markers[numMarkersPerResult * i + 4].text =
        "dist " +
        std::to_string(
            geometryInterface_.getGeometryModel().collisionPairs[i].first) +
        " - " +
        std::to_string(
            geometryInterface_.getGeometryModel().collisionPairs[i].second) +
        ": " + std::to_string(results[i].min_distance);
    markerArray.markers[numMarkersPerResult * i + 4].scale.z = 0.02;
  }

  markerPublisher_->publish(markerArray);
}

}  // namespace ocs2
