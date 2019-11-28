/*
 * AnymalWheelsDummyMRT.cpp
 *
 *  Created on: Nov 27, 2019
 *      Author: Marko Bjelonic
 */

#include <ocs2_quadruped_interface/test/MRT_ROS_Dummy_Quadruped.h>
#include <ros/package.h>
#include <iostream>
#include <string>
#include "ocs2_anymal_wheels/AnymalWheelsInterface.h"
#include "ocs2_anymal_wheels/AnymalWheelsMrtRos.h"

using namespace anymal;
using namespace switched_model;

int main(int argc, char* argv[]) {
  using ocs2_robot_interface_t = AnymalWheelsInterface;
  using mrt_t = AnymalWheelsMrtRos;
  const std::string robotName = "anymal";

  /******************************************************************************************************/
  if (argc <= 1) throw std::runtime_error("No task file specified. Aborting.");
  std::string taskFolder = ros::package::getPath("ocs2_anymal_wheels") + "/config/" + std::string(argv[1]);
  std::string taskFile = taskFolder + "/task.info";
  std::cerr << "Loading task file: " << taskFile << std::endl;

  ocs2_robot_interface_t::Ptr optimizationInterfacePtr(new ocs2_robot_interface_t(taskFolder));
  mrt_t mrt(optimizationInterfacePtr, robotName);

  MRT_ROS_Dummy_Quadruped<16> dummySimulator(optimizationInterfacePtr, mrt, optimizationInterfacePtr->mpcSettings().mrtDesiredFrequency_,
                                             robotName, optimizationInterfacePtr->mpcSettings().mpcDesiredFrequency_);

  dummySimulator.launchNodes(argc, argv);

  // initial state
  ocs2_robot_interface_t::rbd_state_vector_t initRbdState;
  optimizationInterfacePtr->getLoadedInitialState(initRbdState);
  MRT_ROS_Dummy_Quadruped<16>::system_observation_t initObservation;
  initObservation.time() = 0.0;
  optimizationInterfacePtr->computeSwitchedModelState(initRbdState, initObservation.state());
  initObservation.input().setZero();
  initObservation.subsystem() = 15;

  // initial command
  MRT_ROS_Dummy_Quadruped<16>::cost_desired_trajectories_t initCostDesiredTrajectories;

  // time
  auto& timeTrajectory = initCostDesiredTrajectories.desiredTimeTrajectory();
  timeTrajectory.resize(1);
  timeTrajectory[0] = 0.0;

  // State
  auto& stateTrajectory = initCostDesiredTrajectories.desiredStateTrajectory();
  stateTrajectory.resize(1);
  stateTrajectory[0] = initObservation.state();

  // Input
  auto& inputTrajectory = initCostDesiredTrajectories.desiredInputTrajectory();
  inputTrajectory.resize(1);
  inputTrajectory[0] = initObservation.input();

  // run dummy
  dummySimulator.run(initObservation, initCostDesiredTrajectories);
}
