//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingDummyNode.h"

#include <ocs2_ros_interfaces/mrt/LoopshapingDummyObserver.h>

#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>

#include <ocs2_quadruped_interface/QuadrupedLogger.h>
#include <ocs2_quadruped_interface/QuadrupedVisualizer.h>

namespace switched_model_loopshaping {

void quadrupedLoopshapingDummyNode(const rclcpp::Node::SharedPtr &node, const QuadrupedLoopshapingInterface& quadrupedInterface,
                                   double mrtDesiredFrequency, double mpcDesiredFrequency) {
  const std::string robotName = "anymal";

  // MRT
  ocs2::MRT_ROS_Interface mrt(robotName);
  mrt.initRollout(&quadrupedInterface.getRollout());
  mrt.launchNodes(node);

  // Visualization
  auto visualizer = std::make_shared<switched_model::QuadrupedVisualizer>(
      quadrupedInterface.getKinematicModel(), quadrupedInterface.getJointNames(), quadrupedInterface.getBaseName(), node);

  // Logging
  std::string logFileName = "/tmp/ocs2/QuadrupedLoopshapingDummyNodeLog.txt";
  auto logger = std::make_shared<switched_model::QuadrupedLogger>(logFileName, quadrupedInterface.getKinematicModel(),
                                                                  quadrupedInterface.getComModel());

  // Loopshaping observer wrappers
  std::vector<std::shared_ptr<ocs2::DummyObserver>> observers{visualizer, logger};
  auto loopshapingObservers = std::make_shared<ocs2::LoopshapingDummyObserver>(quadrupedInterface.getLoopshapingDefinition(), observers);

  // Dummy MRT
  ocs2::MRT_ROS_Dummy_Loop dummySimulator(mrt, mrtDesiredFrequency, mpcDesiredFrequency);
  dummySimulator.subscribeObservers({loopshapingObservers});

  // initial state
  ocs2::SystemObservation initObservation;
  initObservation.state = quadrupedInterface.getInitialState();
  initObservation.input.setZero(INPUT_DIM);
  initObservation.mode = switched_model::ModeNumber::STANCE;

  // initial command
  ocs2::TargetTrajectories initTargetTrajectories({0.0}, {quadrupedInterface.getQuadrupedInterface().getInitialState()},
                                                  {initObservation.input});

  // run dummy
  dummySimulator.run(initObservation, initTargetTrajectories);
}

}  // namespace switched_model_loopshaping
