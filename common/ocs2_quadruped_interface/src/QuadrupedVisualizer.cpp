//
// Created by rgrandia on 13.02.19.
//

#include "ocs2_quadruped_interface/QuadrupedVisualizer.h"

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"
#include "ocs2_switched_model_interface/core/Rotations.h"

// Visualization helpers
enum class Color { blue, orange, yellow, purple, green, red, black, invisible };

std_msgs::ColorRGBA getColor(double r, double g, double b, double a) {
  std_msgs::ColorRGBA colorMsg;
  colorMsg.r = r;
  colorMsg.g = g;
  colorMsg.b = b;
  colorMsg.a = a;
  return colorMsg;
}

std_msgs::ColorRGBA getColor(Color color) {
  switch (color) {
    case Color::blue:
      return getColor(0, 0.4470, 0.7410, 1.0);
    case Color::orange:
      return getColor(0.8500, 0.3250, 0.0980, 1.0);
    case Color::purple:
      return getColor(0.4940, 0.1840, 0.5560, 1.0);
    case Color::yellow:
      return getColor(0.9290, 0.6940, 0.1250, 1.0);
    case Color::green:
      return getColor(0.4660, 0.6740, 0.1880, 1.0);
    case Color::red:
      return getColor(0.6350, 0.0780, 0.1840, 1.0);
    case Color::black:
      return getColor(0.25, 0.25, 0.25, 1.0);
    case Color::invisible:
    default:
      return getColor(0.0, 0.0, 0.0, 0.0);
  }
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

visualization_msgs::Marker getFootMarker(const Eigen::Vector3d& position, bool contactFlag, Color color) {
  const double footAlphaWhenLifted = 0.3;  // Reduce color intensity when lifted
  const double footMarkerDiameter = 0.03;
  auto footMarker = getSphereMsg(position, color, footMarkerDiameter);
  if (!contactFlag) {
    footMarker.color.a = footAlphaWhenLifted;
  }
  footMarker.ns = "EE Positions";
  return footMarker;
}

visualization_msgs::Marker getForceMarker(const Eigen::Vector3d& force, const Eigen::Vector3d& position, bool contactFlag, Color color) {
  const double forceScale = 1000.0;  // Vector scale in N/m
  auto forceMarker = getArrowToPointMsg(force / forceScale, position, color);
  forceMarker.ns = "EE Forces";
  if (!contactFlag) {
    setInvisible(forceMarker);
  }
  return forceMarker;
}

template <typename ForceIt, typename PositionIt, typename ContactIt>
visualization_msgs::Marker getCenterOfPressureMarker(ForceIt firstForce, ForceIt lastForce, PositionIt positionIt, ContactIt contactIt,
                                                     Color color) {
  const auto copMarkerDiameter = 0.03;

  // Compute center of pressure
  Eigen::Vector3d centerOfPressure = Eigen::Vector3d::Zero();
  double sum_z = 0.0;
  int numContacts = 0;
  for (; firstForce != lastForce; ++firstForce, ++positionIt, ++contactIt) {
    sum_z += firstForce->z();
    centerOfPressure += firstForce->z() * (*positionIt);
    numContacts += (*contactIt) ? 1 : 0;
  }

  // Construct marker
  visualization_msgs::Marker copMarker;
  if (numContacts > 0) {
    centerOfPressure /= sum_z;
    copMarker = getSphereMsg(centerOfPressure, color, copMarkerDiameter);
  } else {
    copMarker = getSphereMsg(centerOfPressure, Color::invisible, copMarkerDiameter);
  }
  copMarker.ns = "Center of Pressure";
  return copMarker;
}

template <typename PositionIt, typename ContactIt>
visualization_msgs::Marker getSupportPolygonMarker(PositionIt firstPos, PositionIt lastPos, ContactIt contactIt, Color color) {
  const double contourLineWidth = 0.005;

  visualization_msgs::Marker lineList;
  lineList.type = visualization_msgs::Marker::LINE_LIST;
  auto numElements = std::distance(firstPos, lastPos);
  lineList.points.reserve(numElements * (numElements - 1) / 2.0);  // Upper bound on the number of lines

  // Loop over all positions
  for (; firstPos != lastPos; ++firstPos, ++contactIt) {
    // For each position, loop over all future positions in the container
    auto nextPos = std::next(firstPos);
    auto nextContact = std::next(contactIt);
    for (; nextPos != lastPos; ++nextPos, ++nextContact) {
      if (*contactIt && *nextContact) {
        // When positions are both marked as in contact, draw a line between the two points
        lineList.points.push_back(getPointMsg(*firstPos));
        lineList.points.push_back(getPointMsg(*nextPos));
      }
    }
  }
  lineList.scale.x = contourLineWidth;
  lineList.color = getColor(color);
  lineList.ns = "Support Polygon";
  return lineList;
}

const double thinLine = 0.005;  // lineWidth in mm
const std::vector<Color> feetColorMap{Color::blue, Color::orange, Color::yellow, Color::purple};

namespace switched_model {

void QuadrupedVisualizer::launchVisualizerNode(ros::NodeHandle& nodeHandle) {
  costDesiredPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>("ocs2_anymal/desiredBaseTrajectory", 1);
  costDesiredPosePublisher_ = nodeHandle.advertise<geometry_msgs::PoseArray>("ocs2_anymal/desiredPoseTrajectory", 1);
  stateOptimizedPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("ocs2_anymal/optimizedStateTrajectory", 1);
  stateOptimizedPosePublisher_ = nodeHandle.advertise<geometry_msgs::PoseArray>("ocs2_anymal/optimizedPoseTrajectory", 1);
  currentStatePublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("ocs2_anymal/currentState", 1);
  currentPosePublisher_ = nodeHandle.advertise<geometry_msgs::PoseArray>("ocs2_anymal/currentPose", 1);

  // Load URDF model
  urdf::Model urdfModel;
  if (!urdfModel.initParam("ocs2_anymal_description")) {
    throw std::runtime_error("[QuadrupedVisualizer] Could not read URDF from: \"ocs2_anymal_description\"");
  }
  KDL::Tree kdlTree;
  kdl_parser::treeFromUrdfModel(urdfModel, kdlTree);

  robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(kdlTree));
  robotStatePublisherPtr_->publishFixedTransforms("");
}

void QuadrupedVisualizer::update(const system_observation_t& observation, const primal_solution_t& primalSolution,
                                 const command_data_t& command) {
  static scalar_t lastTime = std::numeric_limits<scalar_t>::lowest();
  if (observation.time() - lastTime > minPublishTimeDifference_) {
    publishObservation(observation);
    publishDesiredTrajectory(command.mpcCostDesiredTrajectories_);
    publishOptimizedStateTrajectory(primalSolution.timeTrajectory_, primalSolution.stateTrajectory_, primalSolution.eventTimes_,
                                    primalSolution.subsystemsSequence_);
    lastTime = observation.time();
  }
}

void QuadrupedVisualizer::publishObservation(const system_observation_t& observation) {
  // Extract components from state
  const auto time = observation.time();
  const base_coordinate_t comPose = getComPose(observation.state());
  const base_coordinate_t basePose = comModelPtr_->calculateBasePose(comPose);
  const joint_coordinate_t qJoints = getJointPositions(observation.state());
  const Eigen::Matrix3d o_R_b = rotationMatrixBaseToOrigin<scalar_t>(getOrientation(comPose));

  // Compute cartesian state and inputs
  vector_3d_array_t feetPosition;
  vector_3d_array_t feetForce;
  for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
    feetPosition[i] = kinematicModelPtr_->footPositionInOriginFrame(i, basePose, qJoints);
    feetForce[i] = o_R_b * observation.input().template segment<3>(3 * i);
  }

  // Publish
  publishJointTransforms(time, qJoints);
  publishBaseTransform(time, basePose);
  publishCartesianMarkers(time, modeNumber2StanceLeg(observation.subsystem()), comPose, feetPosition, feetForce);
  publishCenterOfMassPose(time, comPose);
}

void QuadrupedVisualizer::publishJointTransforms(double timeStamp, const joint_coordinate_t& jointAngles) const {
  std::map<std::string, double> jointPositions{{"LF_HAA", jointAngles[0]}, {"LF_HFE", jointAngles[1]},  {"LF_KFE", jointAngles[2]},
                                               {"RF_HAA", jointAngles[3]}, {"RF_HFE", jointAngles[4]},  {"RF_KFE", jointAngles[5]},
                                               {"LH_HAA", jointAngles[6]}, {"LH_HFE", jointAngles[7]},  {"LH_KFE", jointAngles[8]},
                                               {"RH_HAA", jointAngles[9]}, {"RH_HFE", jointAngles[10]}, {"RH_KFE", jointAngles[11]}};
  robotStatePublisherPtr_->publishTransforms(jointPositions, ros::Time(timeStamp), "");
  robotStatePublisherPtr_->publishFixedTransforms("");
}

void QuadrupedVisualizer::publishBaseTransform(double timeStamp, const base_coordinate_t& basePose) {
  geometry_msgs::TransformStamped baseToWorldTransform;
  baseToWorldTransform.header.stamp = ros::Time(timeStamp);
  baseToWorldTransform.header.frame_id = "world";
  baseToWorldTransform.child_frame_id = "base";

  const Eigen::Quaternion<scalar_t> q_world_base = quaternionBaseToOrigin<scalar_t>(getOrientation(basePose));
  baseToWorldTransform.transform.rotation = getOrientationMsg(q_world_base);
  baseToWorldTransform.transform.translation = getVectorMsg(getPositionInOrigin(basePose));
  tfBroadcaster_.sendTransform(baseToWorldTransform);
}

void QuadrupedVisualizer::publishTrajectory(const system_observation_array_t& system_observation_array, double speed) {
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

void QuadrupedVisualizer::publishCartesianMarkers(double timeStamp, const contact_flag_t& contactFlags, const base_coordinate_t& comPose,
                                                  const vector_3d_array_t& feetPosition, const vector_3d_array_t& feetForce) const {
  // Reserve message
  const int numberOfCartesianMarkers = 10;
  visualization_msgs::MarkerArray markerArray;
  markerArray.markers.reserve(numberOfCartesianMarkers);

  // Feet positions and Forces
  for (int i = 0; i < NUM_CONTACT_POINTS; ++i) {
    markerArray.markers.emplace_back(getFootMarker(feetPosition[i], contactFlags[i], feetColorMap[i]));
    markerArray.markers.emplace_back(getForceMarker(feetForce[i], feetPosition[i], contactFlags[i], Color::green));
  }

  // Center of pressure
  markerArray.markers.emplace_back(
      getCenterOfPressureMarker(feetForce.begin(), feetForce.end(), feetPosition.begin(), contactFlags.begin(), Color::green));

  // Support polygon
  markerArray.markers.emplace_back(getSupportPolygonMarker(feetPosition.begin(), feetPosition.end(), contactFlags.begin(), Color::black));

  // Give markers an id and a frame
  int id = 0;
  for (auto& m : markerArray.markers) {
    m.header.frame_id = "world";
    m.header.stamp = ros::Time(timeStamp);
    m.id = id++;
  }

  // Publish cartesian markers (minus the CoM Pose)
  currentStatePublisher_.publish(markerArray);
}

void QuadrupedVisualizer::publishCenterOfMassPose(double timeStamp, const base_coordinate_t& comPose) const {
  geometry_msgs::Pose pose;
  pose.position = getPointMsg(getPositionInOrigin(comPose));
  pose.orientation = getOrientationMsg(quaternionBaseToOrigin<double>(getOrientation(comPose)));

  geometry_msgs::PoseArray poseArray;
  poseArray.header.frame_id = "world";
  poseArray.header.stamp = ros::Time(timeStamp);
  poseArray.poses = {pose};

  currentPosePublisher_.publish(poseArray);
}

void QuadrupedVisualizer::computeFeetState(const state_vector_t& state, const input_vector_t& input, vector_3d_array_t& o_feetPosition,
                                           vector_3d_array_t& o_feetVelocity, vector_3d_array_t& o_contactForces) const {
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

void QuadrupedVisualizer::publishDesiredTrajectory(const ocs2::CostDesiredTrajectories& costDesiredTrajectory) const {
  auto& stateTrajectory = costDesiredTrajectory.desiredStateTrajectory();

  // Allocate msg vector
  std::vector<geometry_msgs::Point> desiredComPositionMsg;
  desiredComPositionMsg.reserve(stateTrajectory.size());

  // Extract CoM position and convert to msg.
  std::transform(stateTrajectory.begin(), stateTrajectory.end(), std::back_inserter(desiredComPositionMsg),
                 [](const dynamic_vector_t& state) { return getPointMsg(state.segment(3, 3)); });

  costDesiredPublisher_.publish(getLineMsg(0, std::move(desiredComPositionMsg), Color::green, thinLine));

  // Pose array
  geometry_msgs::PoseArray poseArray;
  poseArray.header.frame_id = "world";
  poseArray.header.stamp = ros::Time::now();
  poseArray.poses.reserve(stateTrajectory.size());

  // Extract and append pose msgs
  std::transform(stateTrajectory.begin(), stateTrajectory.end(), std::back_inserter(poseArray.poses), [&](const dynamic_vector_t& state) {
    const base_coordinate_t comPose = state.head(6);
    geometry_msgs::Pose pose;
    pose.position = getPointMsg(getPositionInOrigin(comPose));
    pose.orientation = getOrientationMsg(quaternionBaseToOrigin<double>(getOrientation(comPose)));
    return pose;
  });
  costDesiredPosePublisher_.publish(poseArray);
}

void QuadrupedVisualizer::publishOptimizedStateTrajectory(const scalar_array_t& mpcTimeTrajectory,
                                                          const state_vector_array_t& mpcStateTrajectory, const scalar_array_t& eventTimes,
                                                          const size_array_t& subsystemSequence) const {
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
  arrayMsg.markers.reserve(6);
  for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
    arrayMsg.markers.push_back(getLineMsg(i, std::move(feetMsgs[i]), feetColorMap[i], thinLine));
    arrayMsg.markers.back().ns = "EE Trajectories";
  }
  arrayMsg.markers.emplace_back(getLineMsg(5, std::move(mpcComPositionMsgs), Color::red, thinLine));
  arrayMsg.markers.back().ns = "CoM Trajectory";

  // Future footholds
  visualization_msgs::Marker sphereList;
  sphereList.type = visualization_msgs::Marker::SPHERE_LIST;
  sphereList.scale.x = 0.03;
  sphereList.ns = "Future footholds";
  sphereList.id = 6;
  sphereList.header.frame_id = "world";
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
          sphereList.colors.push_back(getColor(feetColorMap[i]));
        }
      }
    }
  }
  arrayMsg.markers.push_back(std::move(sphereList));

  stateOptimizedPublisher_.publish(arrayMsg);

  // Pose array
  geometry_msgs::PoseArray poseArray;
  poseArray.header.frame_id = "world";
  poseArray.header.stamp = ros::Time::now();
  poseArray.poses.reserve(mpcStateTrajectory.size());

  // Extract and append pose msgs
  std::transform(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), std::back_inserter(poseArray.poses),
                 [&](const state_vector_t& state) {
                   const base_coordinate_t comPose = getComPose(state);
                   geometry_msgs::Pose pose;
                   pose.position = getPointMsg(getPositionInOrigin(comPose));
                   pose.orientation = getOrientationMsg(quaternionBaseToOrigin<double>(getOrientation(comPose)));
                   return pose;
                 });
  stateOptimizedPosePublisher_.publish(poseArray);
}

}  // namespace switched_model
