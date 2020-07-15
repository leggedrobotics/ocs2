//
// Created by rgrandia on 04.05.20.
//

#include "ocs2_anymal_commands/PoseCommandToCostDesiredRos.h"

#include <ros/package.h>

#include <ocs2_core/misc/LoadData.h>

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/common/RosMsgConversions.h>

#include "ocs2_anymal_commands/TerrainAdaptation.h"

namespace switched_model {

PoseCommandToCostDesiredRos::PoseCommandToCostDesiredRos(const std::string& configFile, ros::NodeHandle& nodeHandle) : localTerrain_() {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(configFile, pt);
  targetDisplacementVelocity = pt.get<scalar_t>("targetDisplacementVelocity");
  targetRotationVelocity = pt.get<scalar_t>("targetRotationVelocity");
  initZHeight = pt.get<scalar_t>("comHeight");
  ocs2::loadData::loadEigenMatrix(configFile, "defaultJointState", defaultJointState);

  // Setup ROS communication
  costDesiredPublisher_ = nodeHandle.advertise<ocs2_msgs::mpc_target_trajectories>("/anymal_mpc_target", 1, false);
  observationSubscriber_ = nodeHandle.subscribe("/anymal_mpc_observation", 1, &PoseCommandToCostDesiredRos::observationCallback, this);
  terrainSubscriber_ = nodeHandle.subscribe("/ocs2_anymal/localTerrain", 1, &PoseCommandToCostDesiredRos::terrainCallback, this);
}

void PoseCommandToCostDesiredRos::observationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(observationMutex_);
  observation_ = msg;
}

void PoseCommandToCostDesiredRos::publishCostDesiredFromCommand(const PoseCommand_t& command) {
  auto deg2rad = [](scalar_t deg) { return (deg * M_PI / 180.0); };

  ocs2::SystemObservation observation;
  if (observation_) {
    std::lock_guard<std::mutex> lock(observationMutex_);
    ocs2::ros_msg_conversions::readObservationMsg(*observation_, observation);
  } else {
    ROS_WARN_STREAM("No observation is received from the MPC node. Make sure the MPC node is running!");
    return;
  }

  // Command to desired Base
  // x, y are relative, z is relative to terrain + default offset;
  vector3_t comPositionDesired{command[0] + observation.state()[3], command[1] + observation.state()[4], command[2] + initZHeight};
  // Roll and pitch are absolute, yaw is relative
  vector3_t comOrientationDesired{deg2rad(command[3]), deg2rad(command[4]), deg2rad(command[5]) + observation.state()[2]};
  const auto desiredTime =
      estimeTimeToTarget(comOrientationDesired.z() - observation.state()[2], comPositionDesired.x() - observation.state()[3],
                         comPositionDesired.y() - observation.state()[4]);

  {  // Terrain adaptation
    std::lock_guard<std::mutex> lock(terrainMutex_);
    comPositionDesired = adaptDesiredPositionHeightToTerrain(comPositionDesired, localTerrain_, comPositionDesired.z());
    comOrientationDesired = alignDesiredOrientationToTerrain(comOrientationDesired, localTerrain_);
  }

  // Trajectory to publish
  ocs2::CostDesiredTrajectories costDesiredTrajectories(2);

  // Desired time trajectory
  scalar_array_t& tDesiredTrajectory = costDesiredTrajectories.desiredTimeTrajectory();
  tDesiredTrajectory.resize(2);
  tDesiredTrajectory[0] = observation.time();
  tDesiredTrajectory[1] = observation.time() + desiredTime;

  // Desired state trajectory
  vector_array_t& xDesiredTrajectory = costDesiredTrajectories.desiredStateTrajectory();
  xDesiredTrajectory.resize(2);
  xDesiredTrajectory[0].resize(STATE_DIM);
  xDesiredTrajectory[0].setZero();
  xDesiredTrajectory[0].segment(0, 12) = observation.state().segment(0, 12);
  xDesiredTrajectory[0].segment(12, 12) = defaultJointState;

  xDesiredTrajectory[1].resize(STATE_DIM);
  xDesiredTrajectory[1].setZero();
  xDesiredTrajectory[1].head(3) = comOrientationDesired;
  // base x, y relative to current state
  xDesiredTrajectory[1].segment(3, 3) = comPositionDesired;
  // target velocities
  xDesiredTrajectory[1].segment(6, 6).setZero();
  // joint angle from initialization
  xDesiredTrajectory[1].segment(12, 12) = defaultJointState;

  // Desired input trajectory
  vector_array_t& uDesiredTrajectory = costDesiredTrajectories.desiredInputTrajectory();
  uDesiredTrajectory.resize(2);
  uDesiredTrajectory[0] = vector_t::Zero(INPUT_DIM);
  uDesiredTrajectory[1] = uDesiredTrajectory[0];

  ocs2_msgs::mpc_target_trajectories mpcTargetTrajectoriesMsg;
  ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(costDesiredTrajectories, mpcTargetTrajectoriesMsg);
  costDesiredPublisher_.publish(mpcTargetTrajectoriesMsg);
}

void PoseCommandToCostDesiredRos::terrainCallback(const visualization_msgs::Marker::ConstPtr& msg) {
  Eigen::Quaterniond orientationTerrainToWorld{msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
                                               msg->pose.orientation.z};

  std::lock_guard<std::mutex> lock(terrainMutex_);
  localTerrain_.positionInWorld = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
  localTerrain_.orientationWorldToTerrain = orientationTerrainToWorld.toRotationMatrix().transpose();
}

scalar_t PoseCommandToCostDesiredRos::estimeTimeToTarget(scalar_t dyaw, scalar_t dx, scalar_t dy) const {
  scalar_t rotationTime = std::abs(dyaw) / targetRotationVelocity;
  scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  scalar_t displacementTime = displacement / targetDisplacementVelocity;

  return std::max(rotationTime, displacementTime);
}

}  // namespace switched_model