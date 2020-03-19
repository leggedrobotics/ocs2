//
// Created by rgrandia on 13.02.19.
//

#include "ocs2_quadruped_interface/QuadrupedVisualizer.h"

#include "ocs2_quadruped_interface/QuadrupedVisualizationHelpers.h"

// Additional messages not in the helpers file
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

// URDF stuff
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

// Switched model conversions
#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"
#include "ocs2_switched_model_interface/core/Rotations.h"

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
    const auto timeStamp = ros::Time(observation.time());
    publishObservation(timeStamp, observation);
    publishDesiredTrajectory(timeStamp, command.mpcCostDesiredTrajectories_);
    publishOptimizedStateTrajectory(timeStamp, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_,
                                    primalSolution.modeSchedule_);
    lastTime = observation.time();
  }
}

void QuadrupedVisualizer::publishObservation(ros::Time timeStamp, const system_observation_t& observation) {
  // Extract components from state
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
  const auto rosTime = ros::Time::now();
  publishJointTransforms(rosTime, qJoints);  // TODO (rgrandia) : using mpc timestamp doesn't work for the TFs
  publishBaseTransform(rosTime, basePose);
  publishCartesianMarkers(timeStamp, modeNumber2StanceLeg(observation.subsystem()), feetPosition, feetForce);
  publishCenterOfMassPose(timeStamp, comPose);
}

void QuadrupedVisualizer::publishJointTransforms(ros::Time timeStamp, const joint_coordinate_t& jointAngles) const {
  std::map<std::string, double> jointPositions{{"LF_HAA", jointAngles[0]}, {"LF_HFE", jointAngles[1]},  {"LF_KFE", jointAngles[2]},
                                               {"RF_HAA", jointAngles[3]}, {"RF_HFE", jointAngles[4]},  {"RF_KFE", jointAngles[5]},
                                               {"LH_HAA", jointAngles[6]}, {"LH_HFE", jointAngles[7]},  {"LH_KFE", jointAngles[8]},
                                               {"RH_HAA", jointAngles[9]}, {"RH_HFE", jointAngles[10]}, {"RH_KFE", jointAngles[11]}};
  robotStatePublisherPtr_->publishTransforms(jointPositions, timeStamp, "");
  robotStatePublisherPtr_->publishFixedTransforms("");
}

void QuadrupedVisualizer::publishBaseTransform(ros::Time timeStamp, const base_coordinate_t& basePose) {
  geometry_msgs::TransformStamped baseToWorldTransform;
  baseToWorldTransform.header = getHeaderMsg(originFrameId_, timeStamp);
  baseToWorldTransform.child_frame_id = "base";

  const Eigen::Quaternion<scalar_t> q_world_base = quaternionBaseToOrigin<scalar_t>(getOrientation(basePose));
  baseToWorldTransform.transform.rotation = getOrientationMsg(q_world_base);
  baseToWorldTransform.transform.translation = getVectorMsg(getPositionInOrigin(basePose));
  tfBroadcaster_.sendTransform(baseToWorldTransform);
}

void QuadrupedVisualizer::publishTrajectory(const system_observation_array_t& system_observation_array, double speed) {
  for (size_t k = 0; k < system_observation_array.size() - 1; k++) {
    double frameDuration = speed * (system_observation_array[k + 1].time() - system_observation_array[k].time());
    double publishDuration =
        timedExecutionInSeconds([&]() { publishObservation(ros::Time(system_observation_array[k].time()), system_observation_array[k]); });
    if (frameDuration > publishDuration) {
      ros::WallDuration(frameDuration - publishDuration).sleep();
    }
  }
}

void QuadrupedVisualizer::publishCartesianMarkers(ros::Time timeStamp, const contact_flag_t& contactFlags,
                                                  const vector_3d_array_t& feetPosition, const vector_3d_array_t& feetForce) const {
  // Reserve message
  const int numberOfCartesianMarkers = 10;
  visualization_msgs::MarkerArray markerArray;
  markerArray.markers.reserve(numberOfCartesianMarkers);

  // Feet positions and Forces
  for (int i = 0; i < NUM_CONTACT_POINTS; ++i) {
    markerArray.markers.emplace_back(
        getFootMarker(feetPosition[i], contactFlags[i], feetColorMap_[i], footMarkerDiameter_, footAlphaWhenLifted_));
    markerArray.markers.emplace_back(getForceMarker(feetForce[i], feetPosition[i], contactFlags[i], Color::green, forceScale_));
  }

  // Center of pressure
  markerArray.markers.emplace_back(getCenterOfPressureMarker(feetForce.begin(), feetForce.end(), feetPosition.begin(), contactFlags.begin(),
                                                             Color::green, copMarkerDiameter_));

  // Support polygon
  markerArray.markers.emplace_back(
      getSupportPolygonMarker(feetPosition.begin(), feetPosition.end(), contactFlags.begin(), Color::black, supportPolygonLineWidth_));

  // Give markers an id and a frame
  assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg(originFrameId_, timeStamp));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

  // Publish cartesian markers (minus the CoM Pose)
  currentStatePublisher_.publish(markerArray);
}

void QuadrupedVisualizer::publishCenterOfMassPose(ros::Time timeStamp, const base_coordinate_t& comPose) const {
  geometry_msgs::Pose pose;
  pose.position = getPointMsg(getPositionInOrigin(comPose));
  pose.orientation = getOrientationMsg(quaternionBaseToOrigin<double>(getOrientation(comPose)));

  geometry_msgs::PoseArray poseArray;
  poseArray.header = getHeaderMsg(originFrameId_, timeStamp);
  poseArray.poses.push_back(std::move(pose));

  currentPosePublisher_.publish(poseArray);
}

void QuadrupedVisualizer::publishDesiredTrajectory(ros::Time timeStamp, const ocs2::CostDesiredTrajectories& costDesiredTrajectory) const {
  const auto& stateTrajectory = costDesiredTrajectory.desiredStateTrajectory();

  // Reserve com messages
  std::vector<geometry_msgs::Point> desiredComPositionMsg;
  desiredComPositionMsg.reserve(stateTrajectory.size());

  // Reserve pose array
  geometry_msgs::PoseArray poseArray;
  poseArray.poses.reserve(stateTrajectory.size());

  std::for_each(stateTrajectory.begin(), stateTrajectory.end(), [&](const dynamic_vector_t& state) {
    // Construct pose msg
    const base_coordinate_t comPose = state.head(6);
    geometry_msgs::Pose pose;
    pose.position = getPointMsg(getPositionInOrigin(comPose));
    pose.orientation = getOrientationMsg(quaternionBaseToOrigin<double>(getOrientation(comPose)));

    // Fill both message containers
    desiredComPositionMsg.push_back(pose.position);
    poseArray.poses.push_back(std::move(pose));
  });

  // Headers
  auto comLineMsg = getLineMsg(std::move(desiredComPositionMsg), Color::green, trajectoryLineWidth_);
  comLineMsg.header = getHeaderMsg(originFrameId_, timeStamp);
  comLineMsg.id = 0;
  poseArray.header = getHeaderMsg(originFrameId_, timeStamp);

  // Publish
  costDesiredPublisher_.publish(comLineMsg);
  costDesiredPosePublisher_.publish(poseArray);
}

void QuadrupedVisualizer::publishOptimizedStateTrajectory(ros::Time timeStamp, const scalar_array_t& mpcTimeTrajectory,
                                                          const state_vector_array_t& mpcStateTrajectory,
                                                          const ocs2::ModeSchedule& modeSchedule) const {
  // Reserve Feet msg
  std::vector<std::vector<geometry_msgs::Point>> feetMsgs(NUM_CONTACT_POINTS);
  std::for_each(feetMsgs.begin(), feetMsgs.end(), [&](std::vector<geometry_msgs::Point>& v) { v.reserve(mpcStateTrajectory.size()); });

  // Reserve Com Msg
  std::vector<geometry_msgs::Point> mpcComPositionMsgs;
  mpcComPositionMsgs.reserve(mpcStateTrajectory.size());

  // Reserve Pose array
  geometry_msgs::PoseArray poseArray;
  poseArray.poses.reserve(mpcStateTrajectory.size());

  // Extract Com and Feet from state
  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const state_vector_array_t::value_type& state) {
    const base_coordinate_t comPose = getComPose(state);
    const base_coordinate_t basePose = comModelPtr_->calculateBasePose(comPose);
    const joint_coordinate_t qJoints = getJointPositions(state);

    // Fill com position and pose msgs
    geometry_msgs::Pose pose;
    pose.position = getPointMsg(getPositionInOrigin(comPose));
    pose.orientation = getOrientationMsg(quaternionBaseToOrigin<double>(getOrientation(comPose)));
    mpcComPositionMsgs.push_back(pose.position);
    poseArray.poses.push_back(std::move(pose));

    // Fill feet msgs
    for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
      const auto o_feetPosition = kinematicModelPtr_->footPositionInOriginFrame(i, basePose, qJoints);
      feetMsgs[i].emplace_back(getPointMsg(o_feetPosition));
    }
  });

  // Convert feet msgs to Array message
  visualization_msgs::MarkerArray markerArray;
  markerArray.markers.reserve(NUM_CONTACT_POINTS + 2);  // 1 trajectory per foot + 1 for the future footholds + 1 for the com trajectory
  for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
    markerArray.markers.emplace_back(getLineMsg(std::move(feetMsgs[i]), feetColorMap_[i], trajectoryLineWidth_));
    markerArray.markers.back().ns = "EE Trajectories";
  }
  markerArray.markers.emplace_back(getLineMsg(std::move(mpcComPositionMsgs), Color::red, trajectoryLineWidth_));
  markerArray.markers.back().ns = "CoM Trajectory";

  // Future footholds
  visualization_msgs::Marker sphereList;
  sphereList.type = visualization_msgs::Marker::SPHERE_LIST;
  sphereList.scale.x = footMarkerDiameter_;
  sphereList.ns = "Future footholds";
  const auto& eventTimes = modeSchedule.eventTimes;
  const auto& subsystemSequence = modeSchedule.modeSequence;
  const double tStart = mpcTimeTrajectory.front();
  const double tEnd = mpcTimeTrajectory.back();
  for (int event = 0; event < eventTimes.size(); ++event) {
    if (tStart < eventTimes[event] && eventTimes[event] < tEnd) {  // Only publish future footholds within the optimized horizon
      const auto postEventContactFlags = modeNumber2StanceLeg(subsystemSequence[event + 1]);
      state_vector_t postEventState;
      ocs2::EigenLinearInterpolation<state_vector_t>::interpolate(eventTimes[event], postEventState, &mpcTimeTrajectory,
                                                                  &mpcStateTrajectory);
      const base_coordinate_t comPose = getComPose(postEventState);
      const base_coordinate_t basePose = comModelPtr_->calculateBasePose(comPose);
      const joint_coordinate_t qJoints = getJointPositions(postEventState);

      for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
        if (postEventContactFlags[i]) {  // If a foot is in contact after an event, a marker is added at that location.
          const auto o_feetPosition = kinematicModelPtr_->footPositionInOriginFrame(i, basePose, qJoints);
          sphereList.points.emplace_back(getPointMsg(o_feetPosition));
          sphereList.colors.push_back(getColor(feetColorMap_[i]));
        }
      }
    }
  }
  markerArray.markers.push_back(std::move(sphereList));

  // Add headers and Id
  assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg(originFrameId_, timeStamp));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());
  poseArray.header = getHeaderMsg(originFrameId_, timeStamp);

  stateOptimizedPublisher_.publish(markerArray);
  stateOptimizedPosePublisher_.publish(poseArray);
}

}  // namespace switched_model
