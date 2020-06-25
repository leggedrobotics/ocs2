//
// Created by rgrandia on 30.04.20.
//

#include "ocs2_quadruped_interface/SwingPlanningVisualizer.h"

#include <geometry_msgs/PoseArray.h>

#include "ocs2_quadruped_interface/QuadrupedVisualizationHelpers.h"

namespace switched_model {

SwingPlanningVisualizer::SwingPlanningVisualizer(std::shared_ptr<const SwingTrajectoryPlanner> swingPlannerPtr, ros::NodeHandle& nodeHandle)
    : swingPlannerPtr_(std::move(swingPlannerPtr)) {
  nominalFootholdPublishers_[0] = nodeHandle.advertise<geometry_msgs::PoseArray>("/ocs2_anymal/swing_planner/nominalFootholds_LF", 1);
  nominalFootholdPublishers_[1] = nodeHandle.advertise<geometry_msgs::PoseArray>("/ocs2_anymal/swing_planner/nominalFootholds_RF", 1);
  nominalFootholdPublishers_[2] = nodeHandle.advertise<geometry_msgs::PoseArray>("/ocs2_anymal/swing_planner/nominalFootholds_LH", 1);
  nominalFootholdPublishers_[3] = nodeHandle.advertise<geometry_msgs::PoseArray>("/ocs2_anymal/swing_planner/nominalFootholds_RH", 1);
}

void SwingPlanningVisualizer::preSolverRun(scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                                           const ocs2::CostDesiredTrajectories& costDesiredTrajectory) {
  const auto timeStamp = ros::Time(initTime);
  for (int leg = 0; leg < NUM_CONTACT_POINTS; leg++) {
    const auto nominalFootholds = swingPlannerPtr_->getNominalFootholds(leg);

    // Reserve pose array
    geometry_msgs::PoseArray poseArray;
    poseArray.poses.reserve(nominalFootholds.size());

    // Convert terrain planes to pose msgs
    std::for_each(nominalFootholds.begin(), nominalFootholds.end(), [&](const ConvexTerrain& foothold) {
      // Construct pose msg
      geometry_msgs::Pose pose;
      pose.position = getPointMsg(foothold.plane.positionInWorld);
      Eigen::Quaterniond terrainOrientation(foothold.plane.orientationWorldToTerrain.transpose());
      pose.orientation = getOrientationMsg(terrainOrientation);

      // Fill message container
      poseArray.poses.push_back(std::move(pose));
    });

    poseArray.header = getHeaderMsg(originFrameId_, timeStamp);
    nominalFootholdPublishers_[leg].publish(poseArray);
  }
}

}  // namespace switched_model
