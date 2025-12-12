//
// Created by rgrandia on 04.05.20.
//

#include "ocs2_anymal_commands/PoseCommandToCostDesiredRos.h"

#include <ocs2_core/misc/LoadData.h>

#include "ocs2_anymal_commands/TerrainAdaptation.h"

namespace switched_model
{

  PoseCommandToCostDesiredRos::PoseCommandToCostDesiredRos(const rclcpp::Node::SharedPtr &node, const std::string &configFile)
  {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(configFile, pt);
    targetDisplacementVelocity = pt.get<scalar_t>("targetDisplacementVelocity");
    targetRotationVelocity = pt.get<scalar_t>("targetRotationVelocity");
    comHeight = pt.get<scalar_t>("comHeight");
    ocs2::loadData::loadEigenMatrix(configFile, "defaultJointState", defaultJointState);

    // Setup ROS communication
    terrainSubscriber_ = node->create_subscription<visualization_msgs::msg::Marker>("/ocs2_anymal/localTerrain", 1, std::bind(&PoseCommandToCostDesiredRos::terrainCallback, this, std::placeholders::_1));
  }

  ocs2::TargetTrajectories PoseCommandToCostDesiredRos::commandLineToTargetTrajectories(const vector_t &commadLineTarget,
                                                                                        const ocs2::SystemObservation &observation) const
  {
    auto deg2rad = [](scalar_t deg)
    { return (deg * M_PI / 180.0); };

    // Command to desired Base
    // x, y are relative, z is relative to terrain + default offset;
    vector3_t comPositionDesired{commadLineTarget(0) + observation.state[3], commadLineTarget(1) + observation.state[4], commadLineTarget(2) + comHeight};
    // Roll and pitch are absolute, yaw is relative
    vector3_t comOrientationDesired{deg2rad(commadLineTarget(3)), deg2rad(commadLineTarget(4)), deg2rad(commadLineTarget(5)) + observation.state[2]};
    const auto desiredTime =
        desiredTimeToTarget(comOrientationDesired.z() - observation.state[2], comPositionDesired.x() - observation.state[3],
                            comPositionDesired.y() - observation.state[4]);

    { // Terrain adaptation
      std::lock_guard<std::mutex> lock(terrainMutex_);
      comPositionDesired = adaptDesiredPositionHeightToTerrain(comPositionDesired, localTerrain_, comPositionDesired.z());
      comOrientationDesired = alignDesiredOrientationToTerrain(comOrientationDesired, localTerrain_);
    }

    // Desired time trajectory
    const ocs2::scalar_array_t tDesiredTrajectory{observation.time, observation.time + desiredTime};

    // Desired state trajectory
    ocs2::vector_array_t xDesiredTrajectory(2, ocs2::vector_t::Zero(STATE_DIM));
    xDesiredTrajectory[0].segment<12>(0) = observation.state.segment<12>(0);
    xDesiredTrajectory[0].segment<12>(12) = defaultJointState;

    xDesiredTrajectory[1].segment<3>(0) = comOrientationDesired;
    // base x, y relative to current state
    xDesiredTrajectory[1].segment<3>(3) = comPositionDesired;
    // target velocities
    xDesiredTrajectory[1].segment<6>(6).setZero();
    // joint angle from initialization
    xDesiredTrajectory[1].segment<12>(12) = defaultJointState;

    // Desired input trajectory
    const ocs2::vector_array_t uDesiredTrajectory(2, ocs2::vector_t::Zero(INPUT_DIM));

    return {tDesiredTrajectory, xDesiredTrajectory, uDesiredTrajectory};
  }

  void PoseCommandToCostDesiredRos::terrainCallback(const visualization_msgs::msg::Marker::ConstSharedPtr &msg)
  {
    Eigen::Quaterniond orientationTerrainToWorld{msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
                                                 msg->pose.orientation.z};

    std::lock_guard<std::mutex> lock(terrainMutex_);
    localTerrain_.positionInWorld = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    localTerrain_.orientationWorldToTerrain = orientationTerrainToWorld.toRotationMatrix().transpose();
  }

  scalar_t PoseCommandToCostDesiredRos::desiredTimeToTarget(scalar_t dyaw, scalar_t dx, scalar_t dy) const
  {
    scalar_t rotationTime = std::abs(dyaw) / targetRotationVelocity;
    scalar_t displacement = std::sqrt(dx * dx + dy * dy);
    scalar_t displacementTime = displacement / targetDisplacementVelocity;

    return std::max(rotationTime, displacementTime);
  }

} // namespace switched_model
