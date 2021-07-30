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

// needs to be included before boost
#include <pinocchio/multibody/geometry.hpp>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_self_collision/PinocchioGeometryInterface.h>
#include <ocs2_self_collision/loadStdVectorOfPair.h>
#include <ocs2_self_collision_visualization/GeometryInterfaceVisualization.h>

#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>

#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

using namespace ocs2;
using namespace mobile_manipulator;

std::unique_ptr<PinocchioInterface> pInterface;
std::shared_ptr<PinocchioGeometryInterface> gInterface;
std::unique_ptr<GeometryInterfaceVisualization> vInterface;

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

  vInterface->publishDistances(q);
}

int main(int argc, char** argv) {
  // Initialize ros node
  ros::init(argc, argv, "distance_visualization");
  ros::NodeHandle nodeHandle;

  const std::string urdfPath = ros::package::getPath("ocs2_mobile_manipulator") + "/urdf/mobile_manipulator.urdf";
  const std::string taskFileFolder = "mpc";
  const std::string taskFile = ros::package::getPath("ocs2_mobile_manipulator") + "/config/" + taskFileFolder + "/task.info";

  std::cerr << "Loading task file: " << taskFile << std::endl;

  pInterface.reset(new PinocchioInterface(MobileManipulatorInterface::buildPinocchioInterface(urdfPath)));

  std::vector<std::pair<size_t, size_t>> selfCollisionObjectPairs;
  std::vector<std::pair<std::string, std::string>> selfCollisionLinkPairs;
  loadData::loadStdVectorOfPair(taskFile, "selfCollisionCost.collisionObjectPairs", selfCollisionObjectPairs);
  loadData::loadStdVectorOfPair(taskFile, "selfCollisionCost.collisionLinkPairs", selfCollisionLinkPairs);
  for (const auto& element : selfCollisionObjectPairs) {
    std::cerr << "[" << element.first << ", " << element.second << "]; ";
  }
  std::cerr << std::endl;
  std::cerr << "Loaded collision link pairs: ";
  for (const auto& element : selfCollisionLinkPairs) {
    std::cerr << "[" << element.first << ", " << element.second << "]; ";
  }
  std::cerr << std::endl;

  gInterface.reset(new PinocchioGeometryInterface(*pInterface, selfCollisionLinkPairs, selfCollisionObjectPairs));

  vInterface.reset(new GeometryInterfaceVisualization(*pInterface, *gInterface, nodeHandle, "base"));

  ros::Subscriber sub = nodeHandle.subscribe("joint_states", 1, &jointStateCallback);

  ros::spin();

  return 0;
}
