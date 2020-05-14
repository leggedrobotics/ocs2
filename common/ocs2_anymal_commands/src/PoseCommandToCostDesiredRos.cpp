//
// Created by rgrandia on 04.05.20.
//

#include "ocs2_anymal_commands/PoseCommandToCostDesiredRos.h"

#include <ros/package.h>

#include <ocs2_core/misc/LoadData.h>

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/common/RosMsgConversions.h>

#include "ocs2_anymal_commands/TerrainAdaptation.h"

namespace switched_model {

PoseCommandToCostDesiredRos::PoseCommandToCostDesiredRos(ros::NodeHandle& nodeHandle, ocs2::LockablePtr<TerrainModel>& terrainPtr) : terrainPptr_(&terrainPtr) {
  // Load settings
  std::string filename = ros::package::getPath("ocs2_anymal_commands") + "/config/targetCommand.info";
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);
  targetDisplacementVelocity = pt.get<scalar_t>("targetDisplacementVelocity");
  targetRotationVelocity = pt.get<scalar_t>("targetRotationVelocity");
  initZHeight = pt.get<scalar_t>("comHeight");
  ocs2::loadData::loadEigenMatrix(filename, "defaultJointState", defaultJointState);

  // Setup ROS communication
  costDesiredPublisher_ = nodeHandle.advertise<ocs2_msgs::mpc_target_trajectories>("/anymal_mpc_target", 1, false);
  observationSubscriber_ = nodeHandle.subscribe("/anymal_mpc_observation", 1,
                                                &PoseCommandToCostDesiredRos::observationCallback, this);
  commandSubscriber_ = nodeHandle.subscribe("/anymal_mpc_pose_command", 1,
                                            &PoseCommandToCostDesiredRos::commandCallback, this);
}

void PoseCommandToCostDesiredRos::observationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(observationMutex_);
  observation_ = msg;
}

void PoseCommandToCostDesiredRos::commandCallback(const ocs2_msgs::mpc_state::ConstPtr& msg) {
  auto deg2rad = [](scalar_t deg) { return (deg * M_PI / 180.0); };

  ocs2_msgs::mpc_state command = *msg;

  ocs2::SystemObservation<STATE_DIM, INPUT_DIM> observation;
  if (observation_) {
    std::lock_guard<std::mutex> lock(observationMutex_);
    ocs2::ros_msg_conversions::readObservationMsg(*observation_, observation);
  } else {
    ROS_WARN_STREAM("No observation is received from the MPC node. Make sure the MPC node is running!");
  }

  // Command to desired Base
  // x, y are relative, z is relative to terrain + default offset;
  vector3_t comPositionDesired{command.value[0] + observation.state()[3], command.value[1] + observation.state()[4], command.value[2] + initZHeight};
  // Roll and pitch are absolute, yaw is relative
  vector3_t comOrientationDesired{deg2rad(command.value[3]), deg2rad(command.value[4]), deg2rad(command.value[5]) + observation.state()[2]};
  const auto desiredTime =
      estimeTimeToTarget(comOrientationDesired.z() - observation.state()[2], comPositionDesired.x() - observation.state()[3],
                         comPositionDesired.y() - observation.state()[4]);

  // Terrain adaptation
  const auto localTerrainPlane = [&]{
    std::lock_guard<ocs2::LockablePtr<TerrainModel>> lock(*terrainPptr_);
    return (*terrainPptr_)->getLocalTerrainAtPositionInWorld(comPositionDesired);
  }();
  comPositionDesired = adaptDesiredPositionHeightToTerrain(comPositionDesired, localTerrainPlane, comPositionDesired.z());
  comOrientationDesired = alignDesiredOrientationToTerrain(comOrientationDesired, localTerrainPlane);

  // Trajectory to publish
  ocs2::CostDesiredTrajectories costDesiredTrajectories(2);

  // Desired time trajectory
  scalar_array_t& tDesiredTrajectory = costDesiredTrajectories.desiredTimeTrajectory();
  tDesiredTrajectory.resize(2);
  tDesiredTrajectory[0] = observation.time();
  tDesiredTrajectory[1] = observation.time() + desiredTime;

  // Desired state trajectory
  ocs2::CostDesiredTrajectories::dynamic_vector_array_t& xDesiredTrajectory = costDesiredTrajectories.desiredStateTrajectory();
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
  ocs2::CostDesiredTrajectories::dynamic_vector_array_t& uDesiredTrajectory = costDesiredTrajectories.desiredInputTrajectory();
  uDesiredTrajectory.resize(2);
  uDesiredTrajectory[0] = ocs2::dynamic_vector_t::Zero(INPUT_DIM);
  uDesiredTrajectory[0][2 + 0] = 80.0;
  uDesiredTrajectory[0][2 + 3] = 80.0;
  uDesiredTrajectory[0][2 + 6] = 80.0;
  uDesiredTrajectory[0][2 + 9] = 80.0;
  uDesiredTrajectory[1] = uDesiredTrajectory[0];

  ocs2_msgs::mpc_target_trajectories mpcTargetTrajectoriesMsg;
  ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(costDesiredTrajectories, mpcTargetTrajectoriesMsg);
  costDesiredPublisher_.publish(mpcTargetTrajectoriesMsg);
}

scalar_t PoseCommandToCostDesiredRos::estimeTimeToTarget(scalar_t dyaw, scalar_t dx, scalar_t dy) const {
  scalar_t rotationTime = std::abs(dyaw) / targetRotationVelocity;
  scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  scalar_t displacementTime = displacement / targetDisplacementVelocity;

  return std::max(rotationTime, displacementTime);
}

}