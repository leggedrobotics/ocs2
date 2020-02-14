/*
 * DummyMRT.cpp
 *
 *  Created on: Apr 10, 2018
 *      Author: farbod
 */

#include <iostream>
#include <string>

#include <ros/package.h>

#include <ocs2_quadruped_interface/MRT_ROS_Quadruped.h>
#include <ocs2_quadruped_interface/test/MRT_ROS_Dummy_Quadruped.h>

#include "ocs2_anymal_bear/AnymalBearInterface.h"

using namespace anymal;
using namespace switched_model;

int main(int argc, char* argv[]) {
  const std::string robotName = "anymal";

  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFolder = ros::package::getPath("ocs2_anymal_bear") + "/config/" + std::string(argv[1]);
  std::cerr << "Loading task file from: " << taskFolder << std::endl;

  std::shared_ptr<AnymalBearInterface> anymalBearInterface(new AnymalBearInterface(taskFolder));
  MRT_ROS_Quadruped<12> mrt(anymalBearInterface, robotName);

  // Visualizer
  std::unique_ptr<QuadrupedXppVisualizer<12>> visualizer(
      new QuadrupedXppVisualizer<12>(anymalBearInterface->getKinematicModel(), anymalBearInterface->getComModel(), robotName, false));

  // Dummy MRT
  MRT_ROS_Dummy_Quadruped<12> dummySimulator(std::move(visualizer), mrt, anymalBearInterface->mpcSettings().mrtDesiredFrequency_,
                                             anymalBearInterface->mpcSettings().mpcDesiredFrequency_);

  dummySimulator.launchNodes(argc, argv);

  // initial state
  MRT_ROS_Dummy_Quadruped<12>::system_observation_t initObservation;
  initObservation.time() = 0.0;
  initObservation.state() = anymalBearInterface->getInitialState();
  initObservation.input().setZero();
  initObservation.subsystem() = 15;

  // initial command
  ocs2::CostDesiredTrajectories initCostDesiredTrajectories({0.0}, {initObservation.state()}, {initObservation.input()});

  // run dummy
  dummySimulator.run(initObservation, initCostDesiredTrajectories);
}
