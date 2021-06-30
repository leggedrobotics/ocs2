#include <ocs2_legged_robot_example/logic/GaitReceiver.h>
#include "ocs2_legged_robot_example/LeggedRobotInterface.h"

#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>

#include <urdf_parser/urdf_parser.h>

int main(int argc, char** argv) {
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() < 5) {
    throw std::runtime_error("No robot name, config folder, target command file, or description name specified. Aborting.");
  }
  const std::string robotName(programArgs[1]);
  const std::string configName(programArgs[2]);
  const std::string targetCommandFile(programArgs[3]);
  const std::string descriptionName("/" + programArgs[4]);

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mpc");
  ros::NodeHandle nodeHandle;

  std::string urdfString;
  if (!ros::param::get(descriptionName, urdfString)) {
    std::cerr << "Param " << descriptionName << " not found; unable to generate urdf" << std::endl;
  }

  ocs2::legged_robot::LeggedRobotInterface leggedRobotInterface(configName, targetCommandFile, urdf::parseURDF(urdfString));

  auto gaitReceiver = std::make_shared<ocs2::legged_robot::GaitReceiver>(
      nodeHandle, leggedRobotInterface.getSwitchedModelModeScheduleManagerPtr()->getGaitSchedule(), robotName);
  auto solverModules = leggedRobotInterface.getSynchronizedModules();
  solverModules.push_back(gaitReceiver);

  // Launch MPC nodes
  auto mpcPtr = leggedRobotInterface.getMpcPtr();
  mpcPtr->getSolverPtr()->setSynchronizedModules(solverModules);
  ocs2::MPC_ROS_Interface mpcNode(*mpcPtr, robotName);
  mpcNode.launchNodes(nodeHandle);

  // Successful exit
  return 0;
}
