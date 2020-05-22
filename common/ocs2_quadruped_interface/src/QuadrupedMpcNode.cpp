//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_quadruped_interface/QuadrupedMpcNode.h"

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>

#include <ocs2_switched_model_interface/logic/GaitReceiver.h>
#include <ocs2_switched_model_interface/terrain/TerrainPlane.h>

#include <ocs2_quadruped_interface/LocalTerrainVisualizer.h>
#include <ocs2_quadruped_interface/QuadrupedSlqMpc.h>
#include <ocs2_quadruped_interface/SwingPlanningVisualizer.h>

namespace switched_model {

void quadrupedMpcNode(ros::NodeHandle& nodeHandle, const QuadrupedInterface& quadrupedInterface, const ocs2::MPC_Settings& mpcSettings,
                      const ocs2::SLQ_Settings& slqSettings) {
  const std::string robotName = "anymal";
  using mpc_ros_t = ocs2::MPC_ROS_Interface<STATE_DIM, INPUT_DIM>;

  auto solverModules = quadrupedInterface.getSynchronizedModules();

  // Gait
  auto gaitReceiver =
      std::make_shared<GaitReceiver>(nodeHandle, quadrupedInterface.getModeScheduleManagerPtr()->getGaitSchedule(), robotName);
  solverModules.push_back(gaitReceiver);

  // Terrain
  auto localTerrainVisualizer =
      std::make_shared<LocalTerrainVisualizer>(quadrupedInterface.getModeScheduleManagerPtr()->getTerrainPtr(), nodeHandle);
  solverModules.push_back(localTerrainVisualizer);

  // Swing planner
  auto swingPlanningVisualizer =
      std::make_shared<SwingPlanningVisualizer>(quadrupedInterface.getModeScheduleManagerPtr()->getSwingTrajectoryPlanner(), nodeHandle);
  solverModules.push_back(swingPlanningVisualizer);

  // launch MPC nodes
  auto mpcPtr = getMpc(quadrupedInterface, mpcSettings, slqSettings);
  mpcPtr->getSolverPtr()->setSynchronizedModules(solverModules);
  mpc_ros_t mpcNode(*mpcPtr, robotName);
  mpcNode.launchNodes(nodeHandle);
}
}  // namespace switched_model
