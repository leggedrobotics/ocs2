//
// Created by rgrandia on 13.02.19.
//

#include "ocs2_quadruped_interface/QuadrupedXppVisualizer.h"

#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_msgs/RobotStateCartesianTrajectory.h>
#include <xpp_msgs/RobotStateJoint.h>
#include <xpp_msgs/topic_names.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "ocs2_switched_model_interface/core/Rotations.h"

// Visualization helpers
enum class Color { red, green, blue };
std_msgs::ColorRGBA getColor(Color color) {
  std_msgs::ColorRGBA colorMsg;
  colorMsg.a = 1.0;
  switch (color) {
    case Color::red:
      colorMsg.r = 1.0;
      break;
    case Color::green:
      colorMsg.g = 1.0;
      break;
    case Color::blue:
      colorMsg.b = 1.0;
      break;
  }
  return colorMsg;
}

visualization_msgs::Marker getLineMsg(int id, std::vector<geometry_msgs::Point>&& points, Color color, double lineWidth) {
  visualization_msgs::Marker line;
  line.header.frame_id = "world";
  line.id = id;
  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.frame_locked = 1U;
  line.scale.x = lineWidth;
  line.color = getColor(color);
  line.points = std::move(points);
  return line;
}

geometry_msgs::Point getPointMsg(double x, double y, double z) {
  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

geometry_msgs::Point getPointMsg(const Eigen::Vector3d& point) {
  return getPointMsg(point[0], point[1], point[2]);
}

geometry_msgs::Vector3 getVectorMsg(double x, double y, double z) {
  geometry_msgs::Vector3 vec;
  vec.x = x;
  vec.y = y;
  vec.z = z;
  return vec;
}

geometry_msgs::Vector3 getVectorMsg(const Eigen::Vector3d& vec) {
  return getVectorMsg(vec[0], vec[1], vec[2]);
}

geometry_msgs::Quaternion getOrientationMsg(const Eigen::Quaterniond& orientation) {
  geometry_msgs::Quaternion orientationMsg;
  orientationMsg.x = orientation.x();
  orientationMsg.y = orientation.y();
  orientationMsg.z = orientation.z();
  orientationMsg.w = orientation.w();
  return orientationMsg;
}

const double thinLine = 0.005;  // lineWidth in mm

namespace switched_model {

void QuadrupedXppVisualizer::launchVisualizerNode(ros::NodeHandle& nodeHandle) {
  visualizationPublisher_ = nodeHandle.advertise<xpp_msgs::RobotStateCartesian>(xpp_msgs::robot_state_desired, 1);
  visualizationJointPublisher_ = nodeHandle.advertise<xpp_msgs::RobotStateJoint>("xpp/joint_anymal_des", 1);
  costDesiredPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>("desiredBaseTrajectory", 1);
  stateOptimizedPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>("optimizedBaseTrajectory", 1);
  feetOptimizedPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("optimizedFeetTrajectories", 1);
}

void QuadrupedXppVisualizer::publishObservation(const system_observation_t& observation) {
  // compute Feet state
  vector_3d_array_t o_feetPositionRef;
  vector_3d_array_t o_feetVelocityRef;
  vector_3d_array_t o_feetAccelerationRef;
  vector_3d_array_t o_feetForceRef;
  computeFeetState(observation.state(), observation.input(), o_feetPositionRef, o_feetVelocityRef, o_feetForceRef);
  for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
    o_feetAccelerationRef[i].setZero();
  }

  // compute RBD state
  const base_coordinate_t comPose = getComPose(observation.state());
  const base_coordinate_t comLocalVelocities = getComLocalVelocities(observation.state());
  const joint_coordinate_t qJoints = getJointPositions(observation.state());

  const base_coordinate_t basePose = comModelPtr_->calculateBasePose(comPose);
  const base_coordinate_t baseLocalVelocities = comModelPtr_->calculateBaseLocalVelocities(comLocalVelocities);

  publishXppVisualizer(observation.time(), basePose, baseLocalVelocities, qJoints, o_feetPositionRef, o_feetVelocityRef,
                       o_feetAccelerationRef, o_feetForceRef);
}

void QuadrupedXppVisualizer::publishTrajectory(const system_observation_array_t& system_observation_array, double speed) {
  for (size_t k = 0; k < system_observation_array.size() - 1; k++) {
    auto start = std::chrono::steady_clock::now();
    double frame_duration = speed * (system_observation_array[k + 1].time() - system_observation_array[k].time());
    publishObservation(system_observation_array[k]);
    auto finish = std::chrono::steady_clock::now();
    double elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(finish - start).count();
    if ((frame_duration - elapsed_seconds) > 0.0) {
      ros::WallDuration(frame_duration - elapsed_seconds).sleep();
    }
  }
}

void QuadrupedXppVisualizer::publishXppVisualizer(scalar_t time, const base_coordinate_t& basePose,
                                                  const base_coordinate_t& baseLocalVelocities, const joint_coordinate_t& jointAngles,
                                                  const vector_3d_array_t& feetPosition, const vector_3d_array_t& feetVelocity,
                                                  const vector_3d_array_t& feetAcceleration, const vector_3d_array_t& feetForce) {
  // Construct the cartesian message
  xpp_msgs::RobotStateCartesian robotStateCartesianMsg;

  const Eigen::Quaternion<scalar_t> q_world_base = quaternionBaseToOrigin<scalar_t>(getOrientation(basePose));
  robotStateCartesianMsg.base.pose.orientation = getOrientationMsg(q_world_base);
  robotStateCartesianMsg.base.pose.position = getPointMsg(getPositionInOrigin(basePose));
  robotStateCartesianMsg.base.twist.angular = getVectorMsg(getAngularVelocity(baseLocalVelocities));
  robotStateCartesianMsg.base.twist.linear = getVectorMsg(getLinearVelocity(baseLocalVelocities));

  robotStateCartesianMsg.time_from_start = ros::Duration(time);

  robotStateCartesianMsg.ee_motion.resize(NUM_CONTACT_POINTS);
  robotStateCartesianMsg.ee_forces.resize(NUM_CONTACT_POINTS);
  robotStateCartesianMsg.ee_contact.resize(NUM_CONTACT_POINTS);
  for (int ee_k = 0; ee_k < NUM_CONTACT_POINTS; ee_k++) {
    robotStateCartesianMsg.ee_motion[ee_k].pos = getPointMsg(feetPosition[ee_k]);
    robotStateCartesianMsg.ee_motion[ee_k].vel = getVectorMsg(feetVelocity[ee_k]);
    robotStateCartesianMsg.ee_motion[ee_k].acc = getVectorMsg(feetAcceleration[ee_k]);
    robotStateCartesianMsg.ee_forces[ee_k] = getVectorMsg(feetForce[ee_k]);
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

void QuadrupedXppVisualizer::computeFeetState(const state_vector_t& state, const input_vector_t& input, vector_3d_array_t& o_feetPosition,
                                              vector_3d_array_t& o_feetVelocity, vector_3d_array_t& o_contactForces) {
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

void QuadrupedXppVisualizer::publishDesiredTrajectory(const cost_desired_trajectories_t& costDesiredTrajectory) {
  auto& stateTrajectory = costDesiredTrajectory.desiredStateTrajectory();

  // Allocate msg vector
  std::vector<geometry_msgs::Point> desiredComPositionMsg;
  desiredComPositionMsg.reserve(stateTrajectory.size());

  // Extract CoM position and convert to msg.
  std::transform(stateTrajectory.begin(), stateTrajectory.end(), std::back_inserter(desiredComPositionMsg),
                 [](const cost_desired_trajectories_t::dynamic_vector_t& state) { return getPointMsg(state.segment(3, 3)); });

  costDesiredPublisher_.publish(getLineMsg(0, std::move(desiredComPositionMsg), Color::green, thinLine));
}

void QuadrupedXppVisualizer::publishOptimizedStateTrajectory(const scalar_array_t& mpcTimeTrajectory,
                                                             const state_vector_array_t& mpcStateTrajectory) {
  // Allocate Com Msg
  std::vector<geometry_msgs::Point> mpcComPositionMsgs;
  mpcComPositionMsgs.reserve(mpcStateTrajectory.size());

  // Allocate Feet msg
  std::vector<std::vector<geometry_msgs::Point>> feetMsgs(NUM_CONTACT_POINTS);
  std::for_each(feetMsgs.begin(), feetMsgs.end(), [&](std::vector<geometry_msgs::Point>& v) { v.reserve(mpcStateTrajectory.size()); });

  // Extract Com and Feet from state
  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const state_vector_array_t::value_type& state) {
    const base_coordinate_t comPose = getComPose(state);
    mpcComPositionMsgs.emplace_back(getPointMsg(getPositionInOrigin(comPose)));

    const base_coordinate_t basePose = comModelPtr_->calculateBasePose(comPose);
    const joint_coordinate_t qJoints = getJointPositions(state);

    for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
      const auto o_feetPosition = kinematicModelPtr_->footPositionInOriginFrame(i, basePose, qJoints);
      feetMsgs[i].emplace_back(getPointMsg(o_feetPosition));
    }
  });

  // Convert feet msgs to Array message
  visualization_msgs::MarkerArray arrayMsg;
  arrayMsg.markers.reserve(NUM_CONTACT_POINTS);
  for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
    arrayMsg.markers.push_back(getLineMsg(i, std::move(feetMsgs[i]), Color::blue, thinLine));
  }

  stateOptimizedPublisher_.publish(getLineMsg(0, std::move(mpcComPositionMsgs), Color::red, thinLine));
  feetOptimizedPublisher_.publish(arrayMsg);
}

}  // namespace switched_model
