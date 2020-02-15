//
// Created by rgrandia on 13.02.20.
//

#include <ros/package.h>

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <ocs2_quadruped_interface/QuadrupedXppVisualizer.h>

#include "ocs2_anymal_bear_loopshaping/AnymalBearLoopshapingInterface.h"
#include "ocs2_anymal_bear_loopshaping/AnymalBearLoopshapingVisualization.h"

int main(int argc, char* argv[]) {
  static constexpr size_t STATE_DIM = 48;
  static constexpr size_t INPUT_DIM = 24;
  static constexpr size_t JOINT_DIM = 12;
  const std::string robotName = "anymal";
  using interface_t = anymal::AnymalBearLoopshapingInterface;
  using vis_t = switched_model::QuadrupedXppVisualizer<JOINT_DIM>;
  using vis_wrapper_t = anymal::AnymalBearLoopshapingVisualization;
  using mrt_t = ocs2::MRT_ROS_Interface<STATE_DIM, INPUT_DIM>;
  using dummy_t = ocs2::MRT_ROS_Dummy_Loop<STATE_DIM, INPUT_DIM>;

  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFolder = ros::package::getPath("ocs2_anymal_bear_loopshaping") + "/config/" +
                           std::string(argv[1]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  std::cerr << "Loading task file from: " << taskFolder << std::endl;

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mrt");
  ros::NodeHandle n;

  // robot interface
  interface_t anymalBearInterface(taskFolder);

  // MRT
  mrt_t mrt(robotName);
  mrt.initRollout(&anymalBearInterface.getRollout());
  mrt.launchNodes(n);

  // Visualization
  std::unique_ptr<vis_t> anymalBearVisualization(
      new vis_t(anymalBearInterface.getKinematicModel(), anymalBearInterface.getComModel(), robotName, n));
  std::shared_ptr<vis_wrapper_t> loopshapingVisualization(
      new vis_wrapper_t(anymalBearInterface.getLoopshapingDefinition(), std::move(anymalBearVisualization)));

  // Dummy MRT
  dummy_t dummySimulator(mrt, anymalBearInterface.mpcSettings().mrtDesiredFrequency_,
                         anymalBearInterface.mpcSettings().mpcDesiredFrequency_);
  dummySimulator.subscribeObservers({loopshapingVisualization});

  // initial state
  mrt_t::system_observation_t initObservation;
  initObservation.state() = anymalBearInterface.getInitialState();
  initObservation.subsystem() = 15;

  // initial command
  ocs2::CostDesiredTrajectories initCostDesiredTrajectories({0.0}, {initObservation.state()}, {initObservation.input()});

  // run dummy
  dummySimulator.run(initObservation, initCostDesiredTrajectories);

  return 0;
}