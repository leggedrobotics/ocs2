#include <ocs2_legged_robot_example/visualization/LeggedRobotDummyVisualization.h>
#include "ocs2_legged_robot_example/LeggedRobotInterface.h"

#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <urdf_parser/urdf_parser.h>

using namespace ocs2;
using namespace legged_robot;

int main(int argc, char** argv) {
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  const std::string taskName(programArgs[1]);

  // Initialize ros node
  ros::init(argc, argv, ROBOT_NAME_ + "_mrt");
  ros::NodeHandle nodeHandle;

  std::string urdfString;
  const std::string descriptionName = "/legged_robot_description";
  if (!ros::param::get(descriptionName, urdfString)) {
    std::cerr << "Param " << descriptionName << " not found; unable to generate urdf" << std::endl;
  }

  LeggedRobotInterface leggedRobotInterface(taskName, urdf::parseURDF(urdfString));

  // MRT
  ocs2::MRT_ROS_Interface mrt(ROBOT_NAME_);
  mrt.initRollout(&leggedRobotInterface.getRollout());
  mrt.launchNodes(nodeHandle);

  // Visualization
  std::shared_ptr<LeggedRobotDummyVisualization> leggedRobotDummyVisualization(
      new LeggedRobotDummyVisualization(nodeHandle, leggedRobotInterface.getPinocchioInterface()));

  // Dummy legged robot
  ocs2::MRT_ROS_Dummy_Loop leggedRobotDummySimulator(mrt, leggedRobotInterface.mpcSettings().mrtDesiredFrequency_,
                                                     leggedRobotInterface.mpcSettings().mpcDesiredFrequency_);
  leggedRobotDummySimulator.subscribeObservers({leggedRobotDummyVisualization});

  // Initial state
  ocs2::SystemObservation initObservation;
  initObservation.state = leggedRobotInterface.getInitialState();
  initObservation.input = vector_t::Zero(INPUT_DIM_);
  initObservation.mode = ModeNumber::STANCE;

  // Initial command
  CostDesiredTrajectories initCostDesiredTrajectories({0.0}, {initObservation.state}, {initObservation.input});

  // run dummy
  leggedRobotDummySimulator.run(initObservation, initCostDesiredTrajectories);

  // Successful exit
  return 0;
}
