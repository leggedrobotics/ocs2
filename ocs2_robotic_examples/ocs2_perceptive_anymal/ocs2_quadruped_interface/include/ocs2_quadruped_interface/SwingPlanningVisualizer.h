//
// Created by rgrandia on 30.04.20.
//

#pragma once

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_ros_interfaces/visualization/VisualizationColors.h>

#include "ocs2_switched_model_interface/foot_planner/SwingTrajectoryPlanner.h"

namespace switched_model {

class SwingPlanningVisualizer : public ocs2::SolverSynchronizedModule {
 public:
  /** Visualization settings (publicly available) */
  std::string frameId_ = "world";  // Frame name all messages are published in
  double arrowScale = 0.05;        // Size of the arrow representing the velocity vector
  switched_model::feet_array_t<ocs2::Color> feetColorMap_ = {ocs2::Color::blue, ocs2::Color::orange, ocs2::Color::yellow,
                                                             ocs2::Color::purple};  // Colors for markers per feet

  SwingPlanningVisualizer(const SwingTrajectoryPlanner& swingTrajectoryPlanner, ros::NodeHandle& nodeHandle);

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ocs2::ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const ocs2::PrimalSolution& primalSolution) override{};

 private:
  const SwingTrajectoryPlanner* swingTrajectoryPlannerPtr_;
  feet_array_t<ros::Publisher> nominalFootholdPublishers_;
  feet_array_t<ros::Publisher> swingTrajectoryPublishers_;
};

}  // namespace switched_model
