//
// Created by rgrandia on 13.02.19.
//

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "ocs2_quadruped_interface/QuadrupedXppVisualizer.h"
#include "ocs2_switched_model_interface/core/Rotations.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void QuadrupedXppVisualizer<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::launchVisualizerNode(ros::NodeHandle& n) {
  visualizationPublisher_ = n.advertise<xpp_msgs::RobotStateCartesian>(xpp_msgs::robot_state_desired, 1);
  visualizationJointPublisher_ = n.advertise<xpp_msgs::RobotStateJoint>("xpp/joint_anymal_des", 1);
  costDesiredPublisher_ = n.advertise<visualization_msgs::Marker>("desiredBaseTrajectory", 1);
  stateOptimizedPublisher_ = n.advertise<visualization_msgs::Marker>("optimizedBaseTrajectory", 1);
  feetOptimizedPublisher_ = n.advertise<visualization_msgs::MarkerArray>("optimizedFeetTrajectories", 1);

  ROS_INFO_STREAM("Waiting for visualization subscriber ...");
  while (ros::ok() && visualizationPublisher_.getNumSubscribers() == 0) {
    ros::Rate(100).sleep();
  }
  ROS_INFO_STREAM("Visualization subscriber is connected.");
}

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void QuadrupedXppVisualizer<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::publishObservation(const system_observation_t& observation) {
  // compute Feet state
  vector_3d_array_t o_feetPositionRef;
  vector_3d_array_t o_feetVelocityRef;
  vector_3d_array_t o_feetAccelerationRef;
  vector_3d_array_t o_feetForceRef;
  computeFeetState(observation.state(), observation.input(), o_feetPositionRef, o_feetVelocityRef, o_feetForceRef);
  for (size_t i = 0; i < 4; i++) {
    o_feetAccelerationRef[i].setZero();
  }

  // compute RBD state
  base_coordinate_t comPose = getComPose(observation.state());
  base_coordinate_t comLocalVelocities = getComLocalVelocities(observation.state());
  joint_coordinate_t qJoints = getJointPositions(observation.state());

  base_coordinate_t basePose = comModelPtr_->calculateBasePose(comPose);
  base_coordinate_t baseLocalVelocities = comModelPtr_->calculateBaseLocalVelocities(comLocalVelocities);

  // contact forces
  for (size_t i = 0; i < 4; i++) {
    o_feetForceRef[i] = observation.input().template segment<3>(3 * i);
  }

  publishXppVisualizer(observation.time(), basePose, baseLocalVelocities, qJoints, o_feetPositionRef, o_feetVelocityRef,
                       o_feetAccelerationRef, o_feetForceRef);
}

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void QuadrupedXppVisualizer<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::publishTrajectory(
    const system_observation_array_t& system_observation_array, double speed) {
  for (size_t k = 0; k < system_observation_array.size() - 1; k++) {
    auto start = std::chrono::steady_clock::now();
    double frame_duration = speed * (system_observation_array[k + 1].time() - system_observation_array[k].time());
    publishObservation(system_observation_array[k]);
    auto finish = std::chrono::steady_clock::now();
    double elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double> >(finish - start).count();
    if ((frame_duration - elapsed_seconds) > 0.0) {
      ros::Duration(frame_duration - elapsed_seconds).sleep();
    }
  }
}

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void QuadrupedXppVisualizer<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::publishXppVisualizer(
    const scalar_t& time, const base_coordinate_t& basePose, const base_coordinate_t& baseLocalVelocities,
    const joint_coordinate_t& jointAngles, const vector_3d_array_t& feetPosition, const vector_3d_array_t& feetVelocity,
    const vector_3d_array_t& feetAcceleration, const vector_3d_array_t& feetForce) {
  const scalar_t minTimeDifference = 10e-3;

  static scalar_t lastTime = 0.0;
  if (time - lastTime < minTimeDifference) {
    return;
  }

  lastTime = time;

  // construct the message
  xpp_msgs::RobotStateCartesian robotStateCartesianMsg;

  const Eigen::Quaternion<scalar_t> q_world_base = quaternionBaseToOrigin<scalar_t>(basePose.template head<3>());
  robotStateCartesianMsg.base.pose.orientation.x = q_world_base.x();
  robotStateCartesianMsg.base.pose.orientation.y = q_world_base.y();
  robotStateCartesianMsg.base.pose.orientation.z = q_world_base.z();
  robotStateCartesianMsg.base.pose.orientation.w = q_world_base.w();
  robotStateCartesianMsg.base.pose.position.x = basePose(3);
  robotStateCartesianMsg.base.pose.position.y = basePose(4);
  robotStateCartesianMsg.base.pose.position.z = basePose(5);

  robotStateCartesianMsg.base.twist.linear.x = baseLocalVelocities(0);
  robotStateCartesianMsg.base.twist.linear.y = baseLocalVelocities(1);
  robotStateCartesianMsg.base.twist.linear.z = baseLocalVelocities(2);
  robotStateCartesianMsg.base.twist.angular.x = baseLocalVelocities(3);
  robotStateCartesianMsg.base.twist.angular.y = baseLocalVelocities(4);
  robotStateCartesianMsg.base.twist.angular.z = baseLocalVelocities(5);

  robotStateCartesianMsg.time_from_start = ros::Duration(time);

  constexpr int numEE = 4;
  robotStateCartesianMsg.ee_motion.resize(numEE);
  robotStateCartesianMsg.ee_forces.resize(numEE);
  robotStateCartesianMsg.ee_contact.resize(numEE);
  for (size_t ee_k = 0; ee_k < numEE; ee_k++) {
    robotStateCartesianMsg.ee_motion[ee_k].pos.x = feetPosition[ee_k](0);
    robotStateCartesianMsg.ee_motion[ee_k].pos.y = feetPosition[ee_k](1);
    robotStateCartesianMsg.ee_motion[ee_k].pos.z = feetPosition[ee_k](2);

    robotStateCartesianMsg.ee_motion[ee_k].vel.x = feetVelocity[ee_k](0);
    robotStateCartesianMsg.ee_motion[ee_k].vel.y = feetVelocity[ee_k](1);
    robotStateCartesianMsg.ee_motion[ee_k].vel.z = feetVelocity[ee_k](2);

    robotStateCartesianMsg.ee_motion[ee_k].acc.x = feetAcceleration[ee_k](0);
    robotStateCartesianMsg.ee_motion[ee_k].acc.y = feetAcceleration[ee_k](1);
    robotStateCartesianMsg.ee_motion[ee_k].acc.z = feetAcceleration[ee_k](2);

    robotStateCartesianMsg.ee_forces[ee_k].x = feetForce[ee_k](0);
    robotStateCartesianMsg.ee_forces[ee_k].y = feetForce[ee_k](1);
    robotStateCartesianMsg.ee_forces[ee_k].z = feetForce[ee_k](2);
  }

  // Joint space message
  xpp_msgs::RobotStateJoint robotStateJointMsg;
  robotStateJointMsg.time_from_start = robotStateCartesianMsg.time_from_start;
  robotStateJointMsg.base = robotStateCartesianMsg.base;
  robotStateJointMsg.ee_contact = robotStateCartesianMsg.ee_contact;
  robotStateJointMsg.joint_state.position = std::vector<double>(jointAngles.data(), jointAngles.data() + jointAngles.size());
  // Attention: Not filling joint velocities or torques

  // Publish
  visualizationPublisher_.publish(robotStateCartesianMsg);
  visualizationJointPublisher_.publish(robotStateJointMsg);
}

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void QuadrupedXppVisualizer<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::computeFeetState(const state_vector_t& state,
                                                                                      const input_vector_t& input,
                                                                                      vector_3d_array_t& o_feetPosition,
                                                                                      vector_3d_array_t& o_feetVelocity,
                                                                                      vector_3d_array_t& o_contactForces) {
  base_coordinate_t comPose = getComPose(state);
  base_coordinate_t comLocalVelocities = getComLocalVelocities(state);
  joint_coordinate_t qJoints = getJointPositions(state);
  joint_coordinate_t dqJoints = getJointVelocities(input);

  base_coordinate_t basePose = comModelPtr_->calculateBasePose(comPose);
  base_coordinate_t baseLocalVelocities = comModelPtr_->calculateBaseLocalVelocities(comLocalVelocities);

  Eigen::Matrix3d o_R_b = rotationMatrixBaseToOrigin<scalar_t>(state.template head<3>());

  for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
    o_feetPosition[i] = kinematicModelPtr_->footPositionInOriginFrame(i, basePose, qJoints);
    o_feetVelocity[i] = kinematicModelPtr_->footVelocityInOriginFrame(i, basePose, baseLocalVelocities, qJoints, dqJoints);
    o_contactForces[i] = o_R_b * input.template segment<3>(3 * i);
  }
}

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void QuadrupedXppVisualizer<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::publishDesiredTrajectory(
    scalar_t startTime, const cost_desired_trajectories_t& costDesiredTrajectory) {
  // Message header
  visualization_msgs::Marker msg;
  msg.header.frame_id = "world";
  msg.id = 0;
  msg.type = visualization_msgs::Marker::LINE_STRIP;
  msg.frame_locked = true;
  msg.scale.x = 0.005;  // used for line width
  msg.color.g = 1.0;
  msg.color.a = 1.0;
  auto& robotStateCartesianMsgs = msg.points;

  // Set up state interpolator
  auto& timeTrajectory = costDesiredTrajectory.desiredTimeTrajectory();
  auto& stateTrajectory = costDesiredTrajectory.desiredStateTrajectory();

  // Evaluation times
  double dt = 0.1;
  double endTime = timeTrajectory.back();
  double t = std::min(startTime, endTime);

  while (t <= endTime) {
    using dynamic_vector_t = typename cost_desired_trajectories_t::dynamic_vector_t;
    dynamic_vector_t state;
    ocs2::EigenLinearInterpolation<dynamic_vector_t>::interpolate(t, state, &timeTrajectory, &stateTrajectory);
    geometry_msgs::Point comPosition;
    comPosition.x = state[3];
    comPosition.y = state[4];
    comPosition.z = state[5];
    robotStateCartesianMsgs.push_back(comPosition);
    t = (t == endTime) ? endTime + dt : std::min(t + dt, endTime);  // make sure endTime is always published, but only once
  }

  costDesiredPublisher_.publish(msg);
}

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void QuadrupedXppVisualizer<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::publishOptimizedStateTrajectory(
    const scalar_array_t& mpcTimeTrajectory, const state_vector_array_t& mpcStateTrajectory) {
  // Message header
  visualization_msgs::Marker msg;
  msg.header.frame_id = "world";
  msg.id = 0;
  msg.type = visualization_msgs::Marker::LINE_STRIP;
  msg.frame_locked = true;
  msg.scale.x = 0.005;  // used for line width
  msg.color.r = 1.0;
  msg.color.a = 1.0;
  auto& robotStateCartesianMsgs = msg.points;

  // Array message header
  visualization_msgs::MarkerArray arrayMsg;
  for (int i = 0; i < 4; i++) {
    visualization_msgs::Marker footMsg;
    footMsg.header.frame_id = "world";
    footMsg.id = i;
    footMsg.type = visualization_msgs::Marker::LINE_STRIP;
    footMsg.frame_locked = true;
    footMsg.scale.x = 0.005;  // used for line width
    footMsg.color.b = 1.0;
    footMsg.color.a = 1.0;
    arrayMsg.markers.push_back(footMsg);
  }

  // compute Feet state
  vector_3d_array_t o_feetPositionRef;
  vector_3d_array_t o_feetVelocityRef;
  vector_3d_array_t o_feetAccelerationRef;
  vector_3d_array_t o_feetForceRef;

  // Evaluation times
  double dt = 0.01;
  double startTime = mpcTimeTrajectory.front();
  double endTime = mpcTimeTrajectory.back();
  double t = std::min(startTime, endTime);

  while (t <= endTime) {
    state_vector_t state;
    ocs2::EigenLinearInterpolation<state_vector_t>::interpolate(t, state, &mpcTimeTrajectory, &mpcStateTrajectory);
    geometry_msgs::Point comPosition;
    comPosition.x = state[3];
    comPosition.y = state[4];
    comPosition.z = state[5];
    robotStateCartesianMsgs.push_back(comPosition);

    computeFeetState(state, input_vector_t::Zero(), o_feetPositionRef, o_feetVelocityRef, o_feetForceRef);
    for (int i = 0; i < 4; i++) {
      geometry_msgs::Point footPosition;
      footPosition.x = o_feetPositionRef[i][0];
      footPosition.y = o_feetPositionRef[i][1];
      footPosition.z = o_feetPositionRef[i][2];
      arrayMsg.markers[i].points.push_back(footPosition);
    }

    t = (t == endTime) ? endTime + dt : std::min(t + dt, endTime);  // make sure endTime is always published, but only once
  }

  stateOptimizedPublisher_.publish(msg);
  feetOptimizedPublisher_.publish(arrayMsg);
}

}  // namespace switched_model
