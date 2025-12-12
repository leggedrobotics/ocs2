//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_quadruped_interface/QuadrupedMpcNode.h"

#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

#include <ocs2_switched_model_interface/logic/GaitReceiver.h>
#include <ocs2_switched_model_interface/terrain/TerrainPlane.h>

#include <ocs2_quadruped_interface/SwingPlanningVisualizer.h>
#include <ocs2_quadruped_interface/TerrainPlaneVisualizer.h>
#include <ocs2_quadruped_interface/TerrainReceiver.h>

namespace switched_model {

void quadrupedMpcNode(const rclcpp::Node::SharedPtr &node, const QuadrupedInterface& quadrupedInterface, std::unique_ptr<ocs2::MPC_BASE> mpcPtr) {
  const std::string robotName = "anymal";

  auto solverModules = quadrupedInterface.getSynchronizedModules();

  // Gait
  auto gaitReceiver =
      std::make_shared<GaitReceiver>(node, quadrupedInterface.getSwitchedModelModeScheduleManagerPtr()->getGaitSchedule(), robotName);
  solverModules.push_back(gaitReceiver);

  // Terrain Receiver
  auto terrainReceiver = std::make_shared<TerrainReceiverSynchronizedModule>(
      quadrupedInterface.getSwitchedModelModeScheduleManagerPtr()->getTerrainModel(), node);
  solverModules.push_back(terrainReceiver);

  // Terrain plane visualization
  auto terrainVisualizer = std::make_shared<TerrainPlaneVisualizerSynchronizedModule>(
      quadrupedInterface.getSwitchedModelModeScheduleManagerPtr()->getSwingTrajectoryPlanner(), node);
  solverModules.push_back(terrainVisualizer);

  // Swing planner
  auto swingPlanningVisualizer = std::make_shared<SwingPlanningVisualizer>(
      quadrupedInterface.getSwitchedModelModeScheduleManagerPtr()->getSwingTrajectoryPlanner(), node);
  solverModules.push_back(swingPlanningVisualizer);

  // reference manager
  auto rosReferenceManagerPtr = std::make_shared<ocs2::RosReferenceManager>(robotName, quadrupedInterface.getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(node);
  mpcPtr->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

  // MPC
  mpcPtr->getSolverPtr()->setSynchronizedModules(solverModules);

  // launch MPC nodes
  ocs2::MPC_ROS_Interface mpcNode(*mpcPtr, robotName);
  mpcNode.launchNodes(node);
}
}  // namespace switched_model
