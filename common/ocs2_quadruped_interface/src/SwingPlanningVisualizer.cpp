//
// Created by rgrandia on 30.04.20.
//

#include "ocs2_quadruped_interface/SwingPlanningVisualizer.h"

#include <geometry_msgs/PoseArray.h>

#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

namespace switched_model {

SwingPlanningVisualizer::SwingPlanningVisualizer(const SwingTrajectoryPlanner& swingTrajectoryPlanner, ros::NodeHandle& nodeHandle)
    : swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner) {
  nominalFootholdPublishers_[0] = nodeHandle.advertise<geometry_msgs::PoseArray>("/ocs2_anymal/swing_planner/nominalFootholds_LF", 1);
  nominalFootholdPublishers_[1] = nodeHandle.advertise<geometry_msgs::PoseArray>("/ocs2_anymal/swing_planner/nominalFootholds_RF", 1);
  nominalFootholdPublishers_[2] = nodeHandle.advertise<geometry_msgs::PoseArray>("/ocs2_anymal/swing_planner/nominalFootholds_LH", 1);
  nominalFootholdPublishers_[3] = nodeHandle.advertise<geometry_msgs::PoseArray>("/ocs2_anymal/swing_planner/nominalFootholds_RH", 1);
}

void SwingPlanningVisualizer::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                                           const ocs2::ReferenceManagerInterface& referenceManager) {
  const auto timeStamp = ros::Time::now();
  for (int leg = 0; leg < NUM_CONTACT_POINTS; leg++) {
    const auto nominalFootholds = swingTrajectoryPlannerPtr_->getNominalFootholds(leg);

    // Reserve pose array
    geometry_msgs::PoseArray poseArray;
    poseArray.poses.reserve(nominalFootholds.size());

    // Convert terrain planes to pose msgs
    std::for_each(nominalFootholds.begin(), nominalFootholds.end(), [&](const ConvexTerrain& foothold) {
      // Construct pose msg
      geometry_msgs::Pose pose;
      pose.position = ocs2::getPointMsg(foothold.plane.positionInWorld);
      Eigen::Quaterniond terrainOrientation(foothold.plane.orientationWorldToTerrain.transpose());
      pose.orientation = ocs2::getOrientationMsg(terrainOrientation);

      // Fill message container
      poseArray.poses.push_back(std::move(pose));
    });

    poseArray.header = ocs2::getHeaderMsg(frameId_, timeStamp);
    nominalFootholdPublishers_[leg].publish(poseArray);
  }
}

}  // namespace switched_model
