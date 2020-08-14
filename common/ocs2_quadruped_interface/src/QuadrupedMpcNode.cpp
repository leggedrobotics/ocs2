//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_quadruped_interface/QuadrupedMpcNode.h"

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>

#include <ocs2_switched_model_interface/logic/GaitReceiver.h>
#include <ocs2_switched_model_interface/terrain/TerrainPlane.h>

#include <ocs2_quadruped_interface/QuadrupedSlqMpc.h>
#include <ocs2_quadruped_interface/SwingPlanningVisualizer.h>
#include <ocs2_quadruped_interface/TerrainPlaneVisualizer.h>

namespace switched_model {

void quadrupedMpcNode(ros::NodeHandle& nodeHandle, const QuadrupedInterface& quadrupedInterface, const ocs2::mpc::Settings& mpcSettings,
                      const ocs2::ddp::Settings& ddpSettings) {
  const std::string robotName = "anymal";

  auto solverModules = quadrupedInterface.getSynchronizedModules();

  // Gait
  auto gaitReceiver =
      std::make_shared<GaitReceiver>(nodeHandle, quadrupedInterface.getModeScheduleManagerPtr()->getGaitSchedule(), robotName);
  solverModules.push_back(gaitReceiver);

  // Terrain
  auto terrainVisualizer = std::make_shared<TerrainPlaneVisualizerSynchronizedModule>(
      quadrupedInterface.getModeScheduleManagerPtr()->getTerrainPtr(), nodeHandle);
  solverModules.push_back(terrainVisualizer);

  // Swing planner
  auto swingPlanningVisualizer =
      std::make_shared<SwingPlanningVisualizer>(quadrupedInterface.getModeScheduleManagerPtr()->getSwingTrajectoryPlanner(), nodeHandle);
  solverModules.push_back(swingPlanningVisualizer);

  // launch MPC nodes
  auto mpcPtr = getMpc(quadrupedInterface, mpcSettings, ddpSettings);
  mpcPtr->getSolverPtr()->setSynchronizedModules(solverModules);
  ocs2::MPC_ROS_Interface mpcNode(*mpcPtr, robotName);
  mpcNode.launchNodes(nodeHandle);
}
}  // namespace switched_model
