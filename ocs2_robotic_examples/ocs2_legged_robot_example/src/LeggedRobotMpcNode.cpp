#include <ocs2_legged_robot_example/logic/GaitReceiver.h>
#include "ocs2_legged_robot_example/LeggedRobotInterface.h"

#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>

#include <urdf_parser/urdf_parser.h>

int main(int argc, char** argv) {
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  const std::string taskName(programArgs[1]);

  // Initialize ros node
  ros::init(argc, argv, ocs2::legged_robot::ROBOT_NAME_ + "_mpc");
  ros::NodeHandle nodeHandle;

  std::string urdfString;
  const std::string descriptionName = "/legged_robot_description";
  if (!ros::param::get(descriptionName, urdfString)) {
    std::cerr << "Param " << descriptionName << " not found; unable to generate urdf" << std::endl;
  }

  ocs2::legged_robot::LeggedRobotInterface leggedRobotInterface(taskName, urdf::parseURDF(urdfString));

  auto gaitReceiver = std::make_shared<ocs2::legged_robot::GaitReceiver>(
      nodeHandle, leggedRobotInterface.getSwitchedModelModeScheduleManagerPtr()->getGaitSchedule(), ocs2::legged_robot::ROBOT_NAME_);
  auto solverModules = leggedRobotInterface.getSynchronizedModules();
  solverModules.push_back(gaitReceiver);

  // Launch MPC nodes
  auto mpcPtr = leggedRobotInterface.getMpcPtr();
  mpcPtr->getSolverPtr()->setSynchronizedModules(solverModules);
  ocs2::MPC_ROS_Interface mpcNode(*mpcPtr, ocs2::legged_robot::ROBOT_NAME_);
  mpcNode.launchNodes(nodeHandle);

  // Successful exit
  return 0;
}
