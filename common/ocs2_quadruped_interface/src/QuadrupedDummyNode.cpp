//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_quadruped_interface/QuadrupedDummyNode.h"

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>

#include <ocs2_quadruped_interface/QuadrupedVisualizer.h>

namespace switched_model {

void quadrupedDummyNode(ros::NodeHandle& nodeHandle, const QuadrupedInterface& quadrupedInterface,
                        const QuadrupedInterface::rollout_base_t* rolloutPtr, double mrtDesiredFrequency, double mpcDesiredFrequency) {
  const std::string robotName = "anymal";
  using vis_t = switched_model::QuadrupedVisualizer;
  using mrt_t = ocs2::MRT_ROS_Interface<STATE_DIM, INPUT_DIM>;
  using dummy_t = ocs2::MRT_ROS_Dummy_Loop<STATE_DIM, INPUT_DIM>;

  // MRT
  mrt_t mrt(robotName);
  if (rolloutPtr != nullptr) {
    mrt.initRollout(rolloutPtr);
  }
  mrt.launchNodes(nodeHandle);

  // Visualization
  auto visualizer = std::make_shared<vis_t>(quadrupedInterface.getKinematicModel(), quadrupedInterface.getComModel(), nodeHandle);

  // Dummy MRT
  dummy_t dummySimulator(mrt, mrtDesiredFrequency, mpcDesiredFrequency);
  dummySimulator.subscribeObservers({visualizer});

  // initial state
  mrt_t::system_observation_t initObservation;
  initObservation.state() = quadrupedInterface.getInitialState();
  initObservation.subsystem() = switched_model::ModeNumber::STANCE;

  // initial command
  ocs2::CostDesiredTrajectories initCostDesiredTrajectories({0.0}, {initObservation.state()}, {initObservation.input()});

  // run dummy
  dummySimulator.run(initObservation, initCostDesiredTrajectories);
}

}  // namespace switched_model
