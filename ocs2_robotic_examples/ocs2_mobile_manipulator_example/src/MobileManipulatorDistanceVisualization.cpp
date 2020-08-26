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

#include <ocs2_mobile_manipulator_example/PinocchioInterface.h>
#include <ocs2_mobile_manipulator_example/PinocchioGeometryInterface.hpp>

#include <ocs2_mobile_manipulator_example/MobileManipulatorVisualizationHelpers.h>

#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

std::shared_ptr<mobile_manipulator::PinocchioInterface<double>> pInterface;
std::shared_ptr<mobile_manipulator::PinocchioGeometryInterface> gInterface;

sensor_msgs::JointState lastMsg;

std::unique_ptr<ros::Publisher> pub;

void jointStateCallback(sensor_msgs::JointStateConstPtr msg) {
  if (lastMsg.position == msg->position) {
    return;
  }
  lastMsg.position = msg->position;

  Eigen::VectorXd q(9);
  q(0) = q(1) = q(2) = 0.0;
  for (size_t i = 3; i < 9; ++i) {
    q(i) = lastMsg.position[i - 3];
  }

  std::cout << "Running distance check" << std::endl;
  std::vector<hpp::fcl::DistanceResult> results = gInterface->computeDistances(q);

  visualization_msgs::MarkerArray markerArray;

  constexpr size_t numMarkersPerResult = 2;

  visualization_msgs::Marker markerTemplate;
  markerTemplate.color = mobile_manipulator::getColor({0, 1, 0}, 1);
  markerTemplate.header.frame_id = "base";
  markerTemplate.header.stamp = ros::Time::now();
  markerTemplate.pose.orientation = mobile_manipulator::getOrientationMsg({1, 0, 0, 0});
  markerArray.markers.resize(results.size() * numMarkersPerResult, markerTemplate);

  for (size_t i = 0; i < results.size(); ++i) {
    std::cout << "Pair " << gInterface->getGeometryModel().collisionPairs[i];
    std::cout << " result " << results[i].min_distance << std::endl;
    std::cout << " results[i].nearest_points[0] " << results[i].nearest_points[0].transpose() << std::endl;
    std::cout << " results[i].nearest_points[1] " << results[i].nearest_points[1].transpose() << std::endl;
    std::cout << std::endl;

    markerArray.markers[numMarkersPerResult * i].type = visualization_msgs::Marker::ARROW;
    markerArray.markers[numMarkersPerResult * i].points.push_back(mobile_manipulator::getPointMsg(results[i].nearest_points[0]));
    markerArray.markers[numMarkersPerResult * i].points.push_back(mobile_manipulator::getPointMsg(results[i].nearest_points[1]));
    markerArray.markers[numMarkersPerResult * i].id = numMarkersPerResult * i;
    markerArray.markers[numMarkersPerResult * i].scale.x = 0.01;
    markerArray.markers[numMarkersPerResult * i + 1].type = visualization_msgs::Marker::SPHERE_LIST;
    markerArray.markers[numMarkersPerResult * i + 1].points.push_back(mobile_manipulator::getPointMsg(results[i].nearest_points[0]));
    markerArray.markers[numMarkersPerResult * i + 1].points.push_back(mobile_manipulator::getPointMsg(results[i].nearest_points[1]));
    markerArray.markers[numMarkersPerResult * i + 1].scale.x = 0.02;
    markerArray.markers[numMarkersPerResult * i + 1].scale.y = 0.02;
    markerArray.markers[numMarkersPerResult * i + 1].scale.z = 0.02;
    markerArray.markers[numMarkersPerResult * i + 1].id = numMarkersPerResult * i + 1;
  }
  std::cout << std::endl;

  pub->publish(markerArray);
}

int main(int argc, char** argv) {
  // Initialize ros node
  ros::init(argc, argv, "distance_visualization");
  ros::NodeHandle nodeHandle;

  const std::string urdfPath = ros::package::getPath("ocs2_mobile_manipulator_example") + "/urdf/mobile_manipulator.urdf";

  pInterface.reset(new mobile_manipulator::PinocchioInterface<double>(urdfPath));
  gInterface.reset(
      new mobile_manipulator::PinocchioGeometryInterface(urdfPath, pInterface, {{0, 4}, {0, 5}, {0, 6}, {0, 7}, {0, 8}, {0, 9}}));

  for (auto obj : gInterface->getGeometryModel().geometryObjects) {
    std::cout << obj.name << std::endl;
  }

  ros::Subscriber sub = nodeHandle.subscribe("joint_states", 1, &jointStateCallback);
  pub.reset(new ros::Publisher(nodeHandle.advertise<visualization_msgs::MarkerArray>("distance_markers", 1, true)));

  ros::spin();

  return 0;
}
