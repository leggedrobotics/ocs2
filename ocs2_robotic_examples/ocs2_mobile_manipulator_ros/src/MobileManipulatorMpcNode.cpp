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


#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>

using namespace ocs2;
using namespace mobile_manipulator;

int main(int argc, char** argv) {
  const std::string robotName = "mobile_manipulator";

  // Initialize ROS2 node
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(robotName + "_mpc",
           rclcpp::NodeOptions()
           .allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true));
  
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  // Get node parameters
  std::string taskFile, libFolder, urdfFile, armSide;
  taskFile = node->get_parameter("taskFile").as_string();
  libFolder = node->get_parameter("libFolder").as_string();
  urdfFile = node->get_parameter("urdfFile").as_string();
  // Check if armSide parameter exists, if not set default to "LEFT"
  if (node->has_parameter("armSide")) {
    armSide = node->get_parameter("armSide").as_string();
    std::cerr << "Set arm side: " << armSide << std::endl;
  } else {
    armSide = "LEFT";
    std::cerr << "armSide parameter not found in launch file, setting default to: " << armSide << std::endl;
  }

  std::cerr << "Loading task file: " << taskFile << std::endl;
  std::cerr << "Loading library folder: " << libFolder << std::endl;
  std::cerr << "Loading urdf file: " << urdfFile << std::endl;
  
  // Robot interface
  MobileManipulatorInterface interface(taskFile, libFolder, urdfFile);

  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<ocs2::RosReferenceManager>(robotName, interface.getReferenceManagerPtr());

  // Check if taskFile contains "aloha" to decide which subscribe and launch method to use
  if (taskFile.find("aloha") != std::string::npos) {
    std::cerr << "For aloha robotic arm, using subscribe_act and launchNodes_mujoco method" << std::endl;
    // Subscribe target from act
    rosReferenceManagerPtr->subscribe_act(node, interface.getPinocchioInterface(), interface.getEeFrameId(), armSide);
    // MPC
    ocs2::GaussNewtonDDP_MPC mpc(interface.mpcSettings(), interface.ddpSettings(), interface.getRollout(),
                                interface.getOptimalControlProblem(), interface.getInitializer());
    mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
    // Launch MPC mujoco ROS node
    MPC_ROS_Interface mpcNode(mpc, robotName);
    mpcNode.launchNodes_mujoco(node);
  } else {
    std::cerr << "For non-aloha, using standard subscribe and launchNodes method" << std::endl;
    // Subscribe target in Rviz
    rosReferenceManagerPtr->subscribe(node);
    // MPC
    ocs2::GaussNewtonDDP_MPC mpc(interface.mpcSettings(), interface.ddpSettings(), interface.getRollout(),
                               interface.getOptimalControlProblem(), interface.getInitializer());
    mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
    // Launch MPC ROS node
    MPC_ROS_Interface mpcNode(mpc, robotName);
    mpcNode.launchNodes(node);
  }

  rclcpp::shutdown();

  // Successful exit
  return 0;
}
