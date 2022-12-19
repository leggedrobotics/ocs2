//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingMpcNode.h"

#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

#include <ocs2_switched_model_interface/logic/GaitReceiver.h>

#include <ocs2_quadruped_interface/SwingPlanningVisualizer.h>
#include <ocs2_quadruped_interface/TerrainPlaneVisualizer.h>
#include <ocs2_quadruped_interface/TerrainReceiver.h>

namespace switched_model_loopshaping {

void quadrupedLoopshapingMpcNode(ros::NodeHandle& nodeHandle, const QuadrupedLoopshapingInterface& quadrupedInterface,
                                 std::unique_ptr<ocs2::MPC_BASE> mpcPtr) {
  const std::string robotName = "anymal";

  auto loopshapingSolverModule = quadrupedInterface.getLoopshapingSynchronizedModule();

  // Gait
  auto gaitReceiver = std::make_shared<switched_model::GaitReceiver>(
      nodeHandle, quadrupedInterface.getQuadrupedInterface().getSwitchedModelModeScheduleManagerPtr()->getGaitSchedule(), robotName);
  loopshapingSolverModule->add(gaitReceiver);

  // Terrain Receiver
  auto terrainReceiver = std::make_shared<switched_model::TerrainReceiverSynchronizedModule>(
      quadrupedInterface.getQuadrupedInterface().getSwitchedModelModeScheduleManagerPtr()->getTerrainModel(), nodeHandle);
  loopshapingSolverModule->add(terrainReceiver);

  // Terrain plane visualization
  auto terrainVisualizer = std::make_shared<switched_model::TerrainPlaneVisualizerSynchronizedModule>(
      quadrupedInterface.getQuadrupedInterface().getSwitchedModelModeScheduleManagerPtr()->getSwingTrajectoryPlanner(), nodeHandle);
  loopshapingSolverModule->add(terrainVisualizer);

  // Swing planner
  auto swingPlanningVisualizer = std::make_shared<switched_model::SwingPlanningVisualizer>(
      quadrupedInterface.getQuadrupedInterface().getSwitchedModelModeScheduleManagerPtr()->getSwingTrajectoryPlanner(), nodeHandle);
  loopshapingSolverModule->add(swingPlanningVisualizer);

  // reference manager
  auto rosReferenceManagerPtr = std::make_shared<ocs2::RosReferenceManager>(robotName, quadrupedInterface.getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nodeHandle);
  mpcPtr->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

  // MPC
  mpcPtr->getSolverPtr()->setSynchronizedModules({loopshapingSolverModule});

  // launch MPC nodes
  ocs2::MPC_ROS_Interface mpcNode(*mpcPtr, robotName);
  mpcNode.launchNodes(nodeHandle);
}

}  // namespace switched_model_loopshaping
