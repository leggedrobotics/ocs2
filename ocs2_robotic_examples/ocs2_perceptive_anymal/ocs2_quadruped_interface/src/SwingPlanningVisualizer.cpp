//
// Created by rgrandia on 30.04.20.
//

#include "ocs2_quadruped_interface/SwingPlanningVisualizer.h"

#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

namespace switched_model
{

  SwingPlanningVisualizer::SwingPlanningVisualizer(const SwingTrajectoryPlanner &swingTrajectoryPlanner, const rclcpp::Node::SharedPtr &node)
      : node_(node), swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner)
  {
    nominalFootholdPublishers_[0] = node->create_publisher<geometry_msgs::msg::PoseArray>("/ocs2_anymal/swing_planner/nominalFootholds_LF", 1);
    nominalFootholdPublishers_[1] = node->create_publisher<geometry_msgs::msg::PoseArray>("/ocs2_anymal/swing_planner/nominalFootholds_RF", 1);
    nominalFootholdPublishers_[2] = node->create_publisher<geometry_msgs::msg::PoseArray>("/ocs2_anymal/swing_planner/nominalFootholds_LH", 1);
    nominalFootholdPublishers_[3] = node->create_publisher<geometry_msgs::msg::PoseArray>("/ocs2_anymal/swing_planner/nominalFootholds_RH", 1);
    swingTrajectoryPublishers_[0] = node->create_publisher<visualization_msgs::msg::MarkerArray>("/ocs2_anymal/swing_planner/trajectory_LF", 1);
    swingTrajectoryPublishers_[1] = node->create_publisher<visualization_msgs::msg::MarkerArray>("/ocs2_anymal/swing_planner/trajectory_RF", 1);
    swingTrajectoryPublishers_[2] = node->create_publisher<visualization_msgs::msg::MarkerArray>("/ocs2_anymal/swing_planner/trajectory_LH", 1);
    swingTrajectoryPublishers_[3] = node->create_publisher<visualization_msgs::msg::MarkerArray>("/ocs2_anymal/swing_planner/trajectory_RH", 1);
  }

  void SwingPlanningVisualizer::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t &currentState,
                                             const ocs2::ReferenceManagerInterface &referenceManager)
  {
    const auto timeStamp = node_->get_clock()->now();

    // Nominal footholds
    for (int leg = 0; leg < NUM_CONTACT_POINTS; leg++)
    {
      const auto nominalFootholds = swingTrajectoryPlannerPtr_->getNominalFootholds(leg);

      // Reserve pose array
      geometry_msgs::msg::PoseArray poseArray;
      poseArray.poses.reserve(nominalFootholds.size());

      // Convert terrain planes to pose msgs
      std::for_each(nominalFootholds.begin(), nominalFootholds.end(), [&](const ConvexTerrain &foothold)
                    {
      // Construct pose msg
      geometry_msgs::msg::Pose pose;
      pose.position = ocs2::getPointMsg(foothold.plane.positionInWorld);
      Eigen::Quaterniond terrainOrientation(foothold.plane.orientationWorldToTerrain.transpose());
      pose.orientation = ocs2::getOrientationMsg(terrainOrientation);

      // Fill message container
      poseArray.poses.push_back(std::move(pose)); });

      poseArray.header = ocs2::getHeaderMsg(frameId_, timeStamp);
      nominalFootholdPublishers_[leg]->publish(poseArray);
    }

    // Feet trajectories
    visualization_msgs::msg::Marker deleteMarker;
    deleteMarker.action = visualization_msgs::msg::Marker::DELETEALL;
    for (int leg = 0; leg < NUM_CONTACT_POINTS; leg++)
    {
      // Initialize and add marker that deletes old visualizations
      visualization_msgs::msg::MarkerArray swingTrajectoriesMessage;
      swingTrajectoriesMessage.markers.reserve(swingTrajectoryPlannerPtr_->getTargetTrajectories().timeTrajectory.size() + 1); // lower bound
      swingTrajectoriesMessage.markers.push_back(deleteMarker);

      for (const auto &t : swingTrajectoryPlannerPtr_->getTargetTrajectories().timeTrajectory)
      {
        if (t < initTime || t > finalTime)
        {
          continue;
        }

        const auto &footPhase = swingTrajectoryPlannerPtr_->getFootPhase(leg, t);
        SwingNode3d node{t, footPhase.getPositionInWorld(t), footPhase.getVelocityInWorld(t)};

        swingTrajectoriesMessage.markers.push_back(ocs2::getArrowAtPointMsg(arrowScale * node.velocity, node.position, feetColorMap_[leg]));
        swingTrajectoriesMessage.markers.back().scale.x = 0.005; // shaft diameter
        swingTrajectoriesMessage.markers.back().scale.y = 0.01;  // arrow-head diameter
        swingTrajectoriesMessage.markers.back().scale.z = 0.02;  // arrow-head length
      }

      ocs2::assignHeader(swingTrajectoriesMessage.markers.begin(), swingTrajectoriesMessage.markers.end(),
                         ocs2::getHeaderMsg(frameId_, timeStamp));
      ocs2::assignIncreasingId(swingTrajectoriesMessage.markers.begin(), swingTrajectoriesMessage.markers.end());
      swingTrajectoryPublishers_[leg]->publish(swingTrajectoriesMessage);
    }
  }

} // namespace switched_model
