//
// Created by rgrandia on 13.02.19.
//

#include "ocs2_quadruped_interface/QuadrupedXppVisualizer.h"

#include <xpp_msgs/topic_names.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"
#include "ocs2_switched_model_interface/core/Rotations.h"

// Visualization helpers
enum class Color { red, green, blue, black, invisible };
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
    case Color::black:
      colorMsg.r = 0.1;
      colorMsg.g = 0.1;
      colorMsg.b = 0.1;
      break;
    case Color::invisible:
      colorMsg.a = 0.0;
      break;
  }
  return colorMsg;
}

void setVisible(visualization_msgs::Marker& marker) {
  marker.color.a = 1.0;
}

void setInvisible(visualization_msgs::Marker& marker) {
  marker.color.a = 0.0;
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
  return getPointMsg(point.x(), point.y(), point.z());
}

geometry_msgs::Vector3 getVectorMsg(double x, double y, double z) {
  geometry_msgs::Vector3 vec;
  vec.x = x;
  vec.y = y;
  vec.z = z;
  return vec;
}

geometry_msgs::Vector3 getVectorMsg(const Eigen::Vector3d& vec) {
  return getVectorMsg(vec.x(), vec.y(), vec.z());
}

geometry_msgs::Quaternion getOrientationMsg(const Eigen::Quaterniond& orientation) {
  geometry_msgs::Quaternion orientationMsg;
  orientationMsg.x = orientation.x();
  orientationMsg.y = orientation.y();
  orientationMsg.z = orientation.z();
  orientationMsg.w = orientation.w();
  return orientationMsg;
}

visualization_msgs::Marker getSphereMsg(const Eigen::Vector3d& point, Color color, double diameter) {
  visualization_msgs::Marker sphere;
  sphere.type = visualization_msgs::Marker::SPHERE;
  sphere.pose.position = getPointMsg(point);
  sphere.scale.x = diameter;
  sphere.scale.y = diameter;
  sphere.scale.z = diameter;
  sphere.color = getColor(color);

  return sphere;
}

visualization_msgs::Marker getArrowToPointMsg(const Eigen::Vector3d& vec, const Eigen::Vector3d& point, Color color) {
  visualization_msgs::Marker arrow;
  arrow.type = visualization_msgs::Marker::ARROW;
  arrow.scale.x = 0.01;  // shaft diameter
  arrow.scale.y = 0.02;  // arrow-head diameter
  arrow.scale.z = 0.06;  // arrow-head length
  arrow.points = {getPointMsg(point - vec), getPointMsg(point)};
  arrow.color = getColor(color);
  return arrow;
}

const double thinLine = 0.005;  // lineWidth in mm

namespace switched_model {

void QuadrupedXppVisualizer::launchVisualizerNode(ros::NodeHandle& nodeHandle) {
  costDesiredPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>("desiredBaseTrajectory", 1);
  stateOptimizedPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>("optimizedBaseTrajectory", 1);
  feetOptimizedPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("optimizedFeetTrajectories", 1);
  rvizMarkerPub_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("ocs2/current_state", 1);

  // Load model from file
  KDL::Tree my_kdl_tree;
  urdf::Model my_urdf_model;
  bool model_ok = my_urdf_model.initParam("ocs2_anymal_description");
  if (!model_ok) {
    ROS_ERROR("Invalid URDF File");
    exit(EXIT_FAILURE);
  }
  ROS_DEBUG("URDF successfully parsed");
  kdl_parser::treeFromUrdfModel(my_urdf_model, my_kdl_tree);
  ROS_DEBUG("Robot tree is ready");

  robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(my_kdl_tree));
  robotStatePublisherPtr_->publishFixedTransforms("");
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

  publishXppVisualizer(observation.time(), modeNumber2StanceLeg(observation.subsystem()), basePose, baseLocalVelocities, qJoints,
                       o_feetPositionRef, o_feetVelocityRef, o_feetAccelerationRef, o_feetForceRef);

  // Publish joint transforms
  std::map<std::string, double> jointPositions{{"LF_HAA", qJoints[0]}, {"LF_HFE", qJoints[1]},  {"LF_KFE", qJoints[2]},
                                               {"RF_HAA", qJoints[3]}, {"RF_HFE", qJoints[4]},  {"RF_KFE", qJoints[5]},
                                               {"LH_HAA", qJoints[6]}, {"LH_HFE", qJoints[7]},  {"LH_KFE", qJoints[8]},
                                               {"RH_HAA", qJoints[9]}, {"RH_HFE", qJoints[10]}, {"RH_KFE", qJoints[11]}};
  robotStatePublisherPtr_->publishTransforms(jointPositions, ros::Time::now(), "");
  robotStatePublisherPtr_->publishFixedTransforms("");

  // Publish base transform
  geometry_msgs::TransformStamped W_X_B_message;
  W_X_B_message.header.stamp = ros::Time::now();
  W_X_B_message.header.frame_id = "world";
  W_X_B_message.child_frame_id = "base";

  const Eigen::Quaternion<scalar_t> q_world_base = quaternionBaseToOrigin<scalar_t>(getOrientation(basePose));
  W_X_B_message.transform.rotation = getOrientationMsg(q_world_base);
  W_X_B_message.transform.translation = getVectorMsg(getPositionInOrigin(basePose));
  tf_broadcaster_.sendTransform(W_X_B_message);
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

void QuadrupedXppVisualizer::publishXppVisualizer(scalar_t time, const contact_flag_t& contactFlags, const base_coordinate_t& basePose,
                                                  const base_coordinate_t& baseLocalVelocities, const joint_coordinate_t& jointAngles,
                                                  const vector_3d_array_t& feetPosition, const vector_3d_array_t& feetVelocity,
                                                  const vector_3d_array_t& feetAcceleration, const vector_3d_array_t& feetForce) {
  // Xpp rip-off
  visualization_msgs::MarkerArray markerArray;

  for (int ee_k = 0; ee_k < NUM_CONTACT_POINTS; ee_k++) {
    // Feet positions
    const auto footMarkerDiameter = 0.03;
    visualization_msgs::Marker footMarker;
    if (contactFlags[ee_k]) {
      footMarker = getSphereMsg(feetPosition[ee_k], Color::blue, footMarkerDiameter);
    } else {
      footMarker = getSphereMsg(feetPosition[ee_k], Color::green, footMarkerDiameter);
    }
    footMarker.ns = "EE_Positions";
    markerArray.markers.push_back(std::move(footMarker));

    // Feet forces
    const double forceScale = 1000.0;  // Vector scale in N/m
    auto forceMarker = getArrowToPointMsg(feetForce[ee_k] / forceScale, feetPosition[ee_k], Color::red);
    forceMarker.ns = "EE_Forces";
    if (!contactFlags[ee_k]) {
      setInvisible(forceMarker);
    }
    markerArray.markers.push_back(std::move(forceMarker));
  }

  // CoP
  const auto copMarkerDiameter = 0.03;
  Eigen::Vector3d centerOfPressure = Eigen::Vector3d::Zero();
  double sum_z = 0.0;
  int numContacts = 0;
  for (int ee_k = 0; ee_k < NUM_CONTACT_POINTS; ee_k++) {
    sum_z += feetForce[ee_k].z();
    centerOfPressure += feetForce[ee_k].z() * feetPosition[ee_k];
    numContacts += contactFlags[ee_k] ? 1 : 0;
  }
  visualization_msgs::Marker copMarker;
  if (numContacts > 0) {
    centerOfPressure /= sum_z;
    copMarker = getSphereMsg(centerOfPressure, Color::red, copMarkerDiameter);
  } else {
    copMarker = getSphereMsg(centerOfPressure, Color::invisible, copMarkerDiameter);
  }
  copMarker.ns = "Center of Pressure";
  markerArray.markers.push_back(std::move(copMarker));

  // Support polygon
  visualization_msgs::Marker lineList;
  lineList.type = visualization_msgs::Marker::LINE_LIST;
  lineList.points.reserve(6);
  // Make connection from every foot to every other foot.
  for (int ee_k = 0; ee_k < NUM_CONTACT_POINTS - 1; ee_k++) {
    for (int ee_j = ee_k + 1; ee_j < NUM_CONTACT_POINTS; ee_j++) {
      if (contactFlags[ee_k] && contactFlags[ee_j]) {
        lineList.points.push_back(getPointMsg(feetPosition[ee_k]));
        lineList.points.push_back(getPointMsg(feetPosition[ee_j]));
      }
    }
  }
  lineList.scale.x = thinLine;
  lineList.color = getColor(Color::black);
  lineList.ns = "Support Polygon";
  markerArray.markers.push_back(std::move(lineList));

  int id = 0;
  for (auto& m : markerArray.markers) {
    m.header.frame_id = "world";
    m.id = id++;
  }
  rvizMarkerPub_.publish(markerArray);
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
                                                             const state_vector_array_t& mpcStateTrajectory,
                                                             const scalar_array_t& eventTimes, const size_array_t& subsystemSequence) {
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

  // Future footholds
  visualization_msgs::Marker sphereList;
  sphereList.type = visualization_msgs::Marker::SPHERE_LIST;
  const double tStart = mpcTimeTrajectory.front();
  const double tEnd = mpcTimeTrajectory.back();
  for (int p = 0; p < subsystemSequence.size(); ++p) {
    if (tStart < eventTimes[p] && eventTimes[p] < tEnd) {  // Only publish future footholds within the optimized horizon
      const auto postEventContactFlags = modeNumber2StanceLeg(subsystemSequence[p]);
      state_vector_t postEventState;
      ocs2::EigenLinearInterpolation<state_vector_t>::interpolate(eventTimes[p], postEventState, &mpcTimeTrajectory, &mpcStateTrajectory);
      const base_coordinate_t comPose = getComPose(postEventState);
      const base_coordinate_t basePose = comModelPtr_->calculateBasePose(comPose);
      const joint_coordinate_t qJoints = getJointPositions(postEventState);

      for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
        if (postEventContactFlags[i]) {
          const auto o_feetPosition = kinematicModelPtr_->footPositionInOriginFrame(i, basePose, qJoints);
          sphereList.points.emplace_back(getPointMsg(o_feetPosition));
        }
      }
    }
  }
  sphereList.scale.x = 0.03;
  sphereList.color = getColor(Color::blue);
  sphereList.ns = "Future footholds";
  sphereList.id = 5;
  sphereList.header.frame_id = "world";
  arrayMsg.markers.push_back(std::move(sphereList));

  stateOptimizedPublisher_.publish(getLineMsg(0, std::move(mpcComPositionMsgs), Color::red, thinLine));
  feetOptimizedPublisher_.publish(arrayMsg);
}

}  // namespace switched_model
