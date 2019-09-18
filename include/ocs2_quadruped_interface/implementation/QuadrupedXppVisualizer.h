//
// Created by rgrandia on 13.02.19.
//

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "ocs2_quadruped_interface/QuadrupedXppVisualizer.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void QuadrupedXppVisualizer<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::launchVisualizerNode(int argc, char** argv) {
  ros::init(argc, argv, robotName_ + "_visualization_node");
  signal(SIGINT, QuadrupedXppVisualizer::sigintHandler);

  ros::NodeHandle n;
  visualizationPublisher_ = n.advertise<xpp_msgs::RobotStateCartesian>(xpp_msgs::robot_state_desired, 1);
  costDesiredPublisher_ = n.advertise<visualization_msgs::Marker>("desiredBaseTrajectory", 100);
  stateOptimizedPublisher_ = n.advertise<visualization_msgs::Marker>("optimizedBaseTrajectory", 100);
  feetOptimizedPublisher_ = n.advertise<visualization_msgs::MarkerArray>("optimizedFeetTrajectories", 100);

  ROS_INFO_STREAM("Waiting for visualization subscriber ...");
  while (ros::ok() && visualizationPublisher_.getNumSubscribers() == 0) {
    ros::Rate(100).sleep();
  }
  ROS_INFO_STREAM("Visualization subscriber is connected.");

  startTime_ = ros::Time::now();

  if (save_rosbag_) {
    robotStateCartesianTrajectoryMsg_.header.stamp = startTime_;
  }
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
  rbd_state_vector_t rbdState;
  ocs2QuadrupedInterfacePtr_->computeRbdModelState(observation.state(), observation.input(), rbdState);

  // contact forces
  for (size_t i = 0; i < 4; i++) {
    o_feetForceRef[i] = observation.input().template segment<3>(3 * i);
  }

  publishXppVisualizer(observation.time(), rbdState.template head<6>(), rbdState.template segment<6>(18), o_feetPositionRef,
                       o_feetVelocityRef, o_feetAccelerationRef, o_feetForceRef);
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
    const vector_3d_array_t& feetPosition, const vector_3d_array_t& feetVelocity, const vector_3d_array_t& feetAcceleration,
    const vector_3d_array_t& feetForce) {
  const scalar_t minTimeDifference = 10e-3;

  static scalar_t lastTime = 0.0;
  if (time - lastTime < minTimeDifference) return;

  lastTime = time;

  // construct the message
  xpp_msgs::RobotStateCartesian point;

  Eigen::Quaternion<scalar_t> qx(cos(basePose(0) / 2), sin(basePose(0) / 2), 0.0, 0.0);
  Eigen::Quaternion<scalar_t> qy(cos(basePose(1) / 2), 0.0, sin(basePose(1) / 2), 0.0);
  Eigen::Quaternion<scalar_t> qz(cos(basePose(2) / 2), 0.0, 0.0, sin(basePose(2) / 2));
  Eigen::Quaternion<scalar_t> qxyz = qz * qy * qx;
  point.base.pose.orientation.x = qxyz.x();
  point.base.pose.orientation.y = qxyz.y();
  point.base.pose.orientation.z = qxyz.z();
  point.base.pose.orientation.w = qxyz.w();
  point.base.pose.position.x = basePose(3);
  point.base.pose.position.y = basePose(4);
  point.base.pose.position.z = basePose(5);

  point.base.twist.linear.x = baseLocalVelocities(0);
  point.base.twist.linear.y = baseLocalVelocities(1);
  point.base.twist.linear.z = baseLocalVelocities(2);
  point.base.twist.angular.x = baseLocalVelocities(3);
  point.base.twist.angular.y = baseLocalVelocities(4);
  point.base.twist.angular.z = baseLocalVelocities(5);

  point.time_from_start = ros::Duration(time);

  constexpr int numEE = 4;
  point.ee_motion.resize(numEE);
  point.ee_forces.resize(numEE);
  point.ee_contact.resize(numEE);
  for (size_t ee_k = 0; ee_k < numEE; ee_k++) {
    point.ee_motion[ee_k].pos.x = feetPosition[ee_k](0);
    point.ee_motion[ee_k].pos.y = feetPosition[ee_k](1);
    point.ee_motion[ee_k].pos.z = feetPosition[ee_k](2);

    point.ee_motion[ee_k].vel.x = feetVelocity[ee_k](0);
    point.ee_motion[ee_k].vel.y = feetVelocity[ee_k](1);
    point.ee_motion[ee_k].vel.z = feetVelocity[ee_k](2);

    point.ee_motion[ee_k].acc.x = feetAcceleration[ee_k](0);
    point.ee_motion[ee_k].acc.y = feetAcceleration[ee_k](1);
    point.ee_motion[ee_k].acc.z = feetAcceleration[ee_k](2);

    point.ee_forces[ee_k].x = feetForce[ee_k](0);
    point.ee_forces[ee_k].y = feetForce[ee_k](1);
    point.ee_forces[ee_k].z = feetForce[ee_k](2);
  }

  visualizationPublisher_.publish(point);

  if (save_rosbag_) {
    const auto stamp = ros::Time(startTime_.toSec() + time);
    try {
      bag_.write("xpp/state_des", stamp, point);
    } catch (const rosbag::BagException& err) {
      std::cerr << "Error writing rosbag message: " << err.what() << std::endl;
    }

    robotStateCartesianTrajectoryMsg_.points.push_back(point);
  }
}

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void QuadrupedXppVisualizer<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::computeFeetState(const state_vector_t& state,
                                                                                      const input_vector_t& input,
                                                                                      vector_3d_array_t& o_feetPosition,
                                                                                      vector_3d_array_t& o_feetVelocity,
                                                                                      vector_3d_array_t& o_contactForces) {
  base_coordinate_t comPose = state.template head<6>();
  base_coordinate_t comLocalVelocities = state.template segment<6>(6);
  joint_coordinate_t qJoints = state.template segment<JOINT_COORD_SIZE>(12);
  joint_coordinate_t dqJoints = input.template segment<JOINT_COORD_SIZE>(12);

  base_coordinate_t basePose;
  ocs2QuadrupedInterfacePtr_->getComModel().calculateBasePose(qJoints, comPose, basePose);
  base_coordinate_t baseLocalVelocities;
  ocs2QuadrupedInterfacePtr_->getComModel().calculateBaseLocalVelocities(qJoints, dqJoints, comLocalVelocities, baseLocalVelocities);

  ocs2QuadrupedInterfacePtr_->getKinematicModel().update(basePose, qJoints);
  Eigen::Matrix3d o_R_b = ocs2QuadrupedInterfacePtr_->getKinematicModel().rotationMatrixOrigintoBase().transpose();

  for (size_t i = 0; i < 4; i++) {
    // calculates foot position in the base frame
    vector_3d_t b_footPosition;
    ocs2QuadrupedInterfacePtr_->getKinematicModel().footPositionBaseFrame(i, b_footPosition);

    // calculates foot position in the origin frame
    o_feetPosition[i] = o_R_b * b_footPosition + basePose.template tail<3>();

    // calculates foot velocity in the base frame
    Eigen::Matrix<scalar_t, 6, JOINT_COORD_SIZE> b_footJacobain;
    ocs2QuadrupedInterfacePtr_->getKinematicModel().footJacobainBaseFrame(i, b_footJacobain);
    vector_3d_t b_footVelocity = (b_footJacobain * dqJoints).template tail<3>();

    // calculates foot velocity in the origin frame
    ocs2QuadrupedInterfacePtr_->getKinematicModel().FromBaseVelocityToInertiaVelocity(o_R_b, baseLocalVelocities, b_footPosition,
                                                                                      b_footVelocity, o_feetVelocity[i]);

    // calculates contact forces in the origin frame
    o_contactForces[i] = o_R_b * input.template segment<3>(3 * i);
  }  // end of i loop
}

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void QuadrupedXppVisualizer<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::sigintHandler(int sig) {
  ROS_INFO_STREAM("Shutting MRT node.");
  ::ros::shutdown();
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
  auto& points = msg.points;

  // Set up state interpolator
  auto& timeTrajectory = costDesiredTrajectory.desiredTimeTrajectory();
  auto& stateTrajectory = costDesiredTrajectory.desiredStateTrajectory();
  ocs2::EigenLinearInterpolation<typename cost_desired_trajectories_t::dynamic_vector_t> stateFunc(&timeTrajectory, &stateTrajectory);

  // Evaluation times
  double dt = 0.1;
  double endTime = timeTrajectory.back();
  double t = std::min(startTime, endTime);

  while (t <= endTime) {
    Eigen::VectorXd state;
    stateFunc.interpolate(t, state);
    geometry_msgs::Point comPosition;
    comPosition.x = state[3];
    comPosition.y = state[4];
    comPosition.z = state[5];
    points.push_back(comPosition);
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
  auto& points = msg.points;

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

  // Set up state interpolator
  ocs2::EigenLinearInterpolation<state_vector_t> stateFunc(&mpcTimeTrajectory, &mpcStateTrajectory);

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
    stateFunc.interpolate(t, state);
    geometry_msgs::Point comPosition;
    comPosition.x = state[3];
    comPosition.y = state[4];
    comPosition.z = state[5];
    points.push_back(comPosition);

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
