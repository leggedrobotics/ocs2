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

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_self_collision/PinocchioGeometryInterface.h>
#include <ocs2_self_collision_visualization/GeometryInterfaceVisualization.h>

#include <ocs2_mobile_manipulator/FactoryFunctions.h>
#include <ocs2_mobile_manipulator/ManipulatorModelInfo.h>
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
  // Get ROS parameters
  std::string urdfPath, taskFile;
  nodeHandle.getParam("/taskFile", taskFile);
  nodeHandle.getParam("/urdfFile", urdfPath);

  // read the task file
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  // read manipulator type
  ManipulatorModelType modelType = mobile_manipulator::loadManipulatorType(taskFile, "model_information.manipulatorModelType");
  // read the joints to make fixed
  std::vector<std::string> removeJointNames;
  loadData::loadStdVector<std::string>(taskFile, "model_information.removeJoints", removeJointNames, true);
  // read the frame names
  std::string baseFrame;
  loadData::loadPtreeValue<std::string>(pt, baseFrame, "model_information.baseFrame", false);

  // create pinocchio interface
  pInterface.reset(new PinocchioInterface(::ocs2::mobile_manipulator::createPinocchioInterface(urdfPath, modelType)));

  std::cerr << "\n #### Model Information:";
  std::cerr << "\n #### =============================================================================\n";
  std::cerr << "\n #### model_information.manipulatorModelType: " << static_cast<int>(modelType);
  std::cerr << "\n #### model_information.removeJoints: ";
  for (const auto& name : removeJointNames) {
    std::cerr << "\"" << name << "\" ";
  }
  std::cerr << "\n #### model_information.baseFrame: \"" << baseFrame << "\"";

  std::vector<std::pair<size_t, size_t>> selfCollisionObjectPairs;
  std::vector<std::pair<std::string, std::string>> selfCollisionLinkPairs;
  loadData::loadStdVectorOfPair(taskFile, "selfCollision.collisionObjectPairs", selfCollisionObjectPairs);
  loadData::loadStdVectorOfPair(taskFile, "selfCollision.collisionLinkPairs", selfCollisionLinkPairs);
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

  vInterface.reset(new GeometryInterfaceVisualization(*pInterface, *gInterface, nodeHandle, baseFrame));

  ros::Subscriber sub = nodeHandle.subscribe("joint_states", 1, &jointStateCallback);

  ros::spin();

  return 0;
}
