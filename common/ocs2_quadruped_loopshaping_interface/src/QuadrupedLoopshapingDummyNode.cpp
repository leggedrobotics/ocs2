//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingDummyNode.h"

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingVisualizer.h>

namespace switched_model_loopshaping {

void quadrupedLoopshapingDummyNode(ros::NodeHandle& nodeHandle, const QuadrupedLoopshapingInterface& quadrupedInterface,
                                   double mrtDesiredFrequency, double mpcDesiredFrequency) {
  const std::string robotName = "anymal";

  // MRT
  ocs2::MRT_ROS_Interface mrt(robotName);
  mrt.initRollout(&quadrupedInterface.getRollout());
  mrt.launchNodes(nodeHandle);

  // Visualization
  std::unique_ptr<switched_model::QuadrupedVisualizer> visualizer(
      new switched_model::QuadrupedVisualizer(quadrupedInterface.getKinematicModel(), quadrupedInterface.getComModel(), nodeHandle));
  auto loopshapingVisualizer =
      std::make_shared<QuadrupedLoopshapingVisualizer>(quadrupedInterface.getLoopshapingDefinition(), std::move(visualizer));

  // Dummy MRT
  ocs2::MRT_ROS_Dummy_Loop dummySimulator(mrt, mrtDesiredFrequency, mpcDesiredFrequency);
  dummySimulator.subscribeObservers({loopshapingVisualizer});

  // initial state
  ocs2::SystemObservation initObservation;
  initObservation.state() = quadrupedInterface.getInitialState();
  initObservation.input().setZero(INPUT_DIM);
  initObservation.subsystem() = switched_model::ModeNumber::STANCE;

  // initial command
  ocs2::CostDesiredTrajectories initCostDesiredTrajectories({0.0}, {initObservation.state()}, {initObservation.input()});

  // run dummy
  dummySimulator.run(initObservation, initCostDesiredTrajectories);
}

}  // namespace switched_model_loopshaping
