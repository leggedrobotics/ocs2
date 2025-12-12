//
// Created by rgrandia on 04.05.20.
//

#pragma once

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>

#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/terrain/TerrainModel.h>

namespace switched_model
{

  class PoseCommandToCostDesiredRos
  {
  public:
    scalar_t targetDisplacementVelocity;
    scalar_t targetRotationVelocity;
    scalar_t comHeight;
    joint_coordinate_t defaultJointState;

    PoseCommandToCostDesiredRos(const rclcpp::Node::SharedPtr &node, const std::string &configFile);

    ocs2::TargetTrajectories commandLineToTargetTrajectories(const vector_t &commadLineTarget, const ocs2::SystemObservation &observation) const;

  private:
    scalar_t desiredTimeToTarget(scalar_t dyaw, scalar_t dx, scalar_t dy) const;

    void terrainCallback(const visualization_msgs::msg::Marker::ConstSharedPtr &msg);

    TerrainPlane localTerrain_;
    mutable std::mutex terrainMutex_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr terrainSubscriber_;
  };

}
