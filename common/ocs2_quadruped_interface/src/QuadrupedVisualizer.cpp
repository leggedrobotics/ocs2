//
// Created by rgrandia on 13.02.19.
//

#include "ocs2_quadruped_interface/QuadrupedVisualizer.h"

#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

#include <ocs2_core/misc/LinearInterpolation.h>

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
  costDesiredBasePositionPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>("/ocs2_anymal/desiredBaseTrajectory", 1);
  costDesiredBasePosePublisher_ = nodeHandle.advertise<geometry_msgs::PoseArray>("/ocs2_anymal/desiredPoseTrajectory", 1);
  costDesiredBaseAngVelocityPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/ocs2_anymal/desiredAngVelTrajectory", 1);
  costDesiredBaseVelocityPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/ocs2_anymal/desiredVelTrajectory", 1);
  costDesiredFeetPositionPublishers_[0] = nodeHandle.advertise<visualization_msgs::Marker>("/ocs2_anymal/desiredFeetTrajectory/LF", 1);
  costDesiredFeetPositionPublishers_[1] = nodeHandle.advertise<visualization_msgs::Marker>("/ocs2_anymal/desiredFeetTrajectory/RF", 1);
  costDesiredFeetPositionPublishers_[2] = nodeHandle.advertise<visualization_msgs::Marker>("/ocs2_anymal/desiredFeetTrajectory/LH", 1);
  costDesiredFeetPositionPublishers_[3] = nodeHandle.advertise<visualization_msgs::Marker>("/ocs2_anymal/desiredFeetTrajectory/RH", 1);
  costDesiredFeetPosePublishers_[0] = nodeHandle.advertise<geometry_msgs::PoseArray>("/ocs2_anymal/desiredFeetPoseTrajectory/LF", 1);
  costDesiredFeetPosePublishers_[1] = nodeHandle.advertise<geometry_msgs::PoseArray>("/ocs2_anymal/desiredFeetPoseTrajectory/RF", 1);
  costDesiredFeetPosePublishers_[2] = nodeHandle.advertise<geometry_msgs::PoseArray>("/ocs2_anymal/desiredFeetPoseTrajectory/LH", 1);
  costDesiredFeetPosePublishers_[3] = nodeHandle.advertise<geometry_msgs::PoseArray>("/ocs2_anymal/desiredFeetPoseTrajectory/RH", 1);
  costDesiredFeetVelocityPublishers_[0] =
      nodeHandle.advertise<visualization_msgs::MarkerArray>("/ocs2_anymal/desiredFeetVelTrajectory/LF", 1);
  costDesiredFeetVelocityPublishers_[1] =
      nodeHandle.advertise<visualization_msgs::MarkerArray>("/ocs2_anymal/desiredFeetVelTrajectory/RF", 1);
  costDesiredFeetVelocityPublishers_[2] =
      nodeHandle.advertise<visualization_msgs::MarkerArray>("/ocs2_anymal/desiredFeetVelTrajectory/LH", 1);
  costDesiredFeetVelocityPublishers_[3] =
      nodeHandle.advertise<visualization_msgs::MarkerArray>("/ocs2_anymal/desiredFeetVelTrajectory/RH", 1);
  stateOptimizedPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/ocs2_anymal/optimizedStateTrajectory", 1);
  stateOptimizedPosePublisher_ = nodeHandle.advertise<geometry_msgs::PoseArray>("/ocs2_anymal/optimizedPoseTrajectory", 1);
  stateOptimizedFeetPosePublishers_[0] = nodeHandle.advertise<geometry_msgs::PoseArray>("/ocs2_anymal/optimizedFeetPoseTrajectory/LF", 1);
  stateOptimizedFeetPosePublishers_[1] = nodeHandle.advertise<geometry_msgs::PoseArray>("/ocs2_anymal/optimizedFeetPoseTrajectory/RF", 1);
  stateOptimizedFeetPosePublishers_[2] = nodeHandle.advertise<geometry_msgs::PoseArray>("/ocs2_anymal/optimizedFeetPoseTrajectory/LH", 1);
  stateOptimizedFeetPosePublishers_[3] = nodeHandle.advertise<geometry_msgs::PoseArray>("/ocs2_anymal/optimizedFeetPoseTrajectory/RH", 1);
  currentStatePublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/ocs2_anymal/currentState", 1);
  currentFeetPosesPublisher_ = nodeHandle.advertise<geometry_msgs::PoseArray>("/ocs2_anymal/currentFeetPoses", 1);
  currentCollisionSpheresPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/ocs2_anymal/currentCollisionSpheres", 1);

  // Load URDF model
  urdf::Model urdfModel;
  if (!urdfModel.initParam("ocs2_anymal_description")) {
    std::cerr << "[QuadrupedVisualizer] Could not read URDF from: \"ocs2_anymal_description\"" << std::endl;
  } else {
    KDL::Tree kdlTree;
    kdl_parser::treeFromUrdfModel(urdfModel, kdlTree);

    robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(kdlTree));
    robotStatePublisherPtr_->publishFixedTransforms(true);
  }
}

void QuadrupedVisualizer::update(const ocs2::SystemObservation& observation, const ocs2::PrimalSolution& primalSolution,
                                 const ocs2::CommandData& command) {
  if (observation.time - lastTime_ > minPublishTimeDifference_) {
    const auto timeStamp = ros::Time::now();
    publishObservation(timeStamp, observation);
    publishDesiredTrajectory(timeStamp, command.mpcTargetTrajectories_);
    publishOptimizedStateTrajectory(timeStamp, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_,
                                    primalSolution.modeSchedule_);
    lastTime_ = observation.time;
  }
}

void QuadrupedVisualizer::publishObservation(ros::Time timeStamp, const ocs2::SystemObservation& observation) {
  // Extract components from state
  const state_vector_t switchedState = observation.state.head(STATE_DIM);
  const base_coordinate_t basePose = getBasePose(switchedState);
  const joint_coordinate_t qJoints = getJointPositions(state_vector_t(observation.state));
  const Eigen::Matrix3d o_R_b = rotationMatrixBaseToOrigin<scalar_t>(getOrientation(basePose));

  // Compute cartesian state and inputs
  feet_array_t<vector3_t> feetPosition;
  feet_array_t<vector3_t> feetForce;
  feet_array_t<Eigen::Quaternion<scalar_t>> feetOrientations;
  for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
    feetPosition[i] = kinematicModelPtr_->footPositionInOriginFrame(i, basePose, qJoints);
    feetForce[i] = o_R_b * observation.input.segment<3>(3 * i);
    feetOrientations[i] = Eigen::Quaternion<scalar_t>(kinematicModelPtr_->footOrientationInOriginFrame(i, basePose, qJoints));
  }

  // Publish
  publishJointTransforms(timeStamp, qJoints);
  publishBaseTransform(timeStamp, basePose);
  publishCartesianMarkers(timeStamp, modeNumber2StanceLeg(observation.mode), feetPosition, feetForce);
  publishEndEffectorPoses(timeStamp, feetPosition, feetOrientations);
  publishCollisionSpheres(timeStamp, basePose, qJoints);
}

void QuadrupedVisualizer::publishJointTransforms(ros::Time timeStamp, const joint_coordinate_t& jointAngles) const {
  if (robotStatePublisherPtr_ != nullptr) {
    std::map<std::string, double> jointPositions{{"LF_HAA", jointAngles[0]}, {"LF_HFE", jointAngles[1]},  {"LF_KFE", jointAngles[2]},
                                                 {"RF_HAA", jointAngles[3]}, {"RF_HFE", jointAngles[4]},  {"RF_KFE", jointAngles[5]},
                                                 {"LH_HAA", jointAngles[6]}, {"LH_HFE", jointAngles[7]},  {"LH_KFE", jointAngles[8]},
                                                 {"RH_HAA", jointAngles[9]}, {"RH_HFE", jointAngles[10]}, {"RH_KFE", jointAngles[11]}};
    robotStatePublisherPtr_->publishTransforms(jointPositions, timeStamp);
  }
}

void QuadrupedVisualizer::publishBaseTransform(ros::Time timeStamp, const base_coordinate_t& basePose) {
  if (robotStatePublisherPtr_ != nullptr) {
    geometry_msgs::TransformStamped baseToWorldTransform;
    baseToWorldTransform.header = ocs2::getHeaderMsg(frameId_, timeStamp);
    baseToWorldTransform.child_frame_id = "base";

    const Eigen::Quaternion<scalar_t> q_world_base = quaternionBaseToOrigin<scalar_t>(getOrientation(basePose));
    baseToWorldTransform.transform.rotation = ocs2::getOrientationMsg(q_world_base);
    baseToWorldTransform.transform.translation = ocs2::getVectorMsg(getPositionInOrigin(basePose));
    tfBroadcaster_.sendTransform(baseToWorldTransform);
  }
}

void QuadrupedVisualizer::publishTrajectory(const std::vector<ocs2::SystemObservation>& system_observation_array, double speed) {
  for (size_t k = 0; k < system_observation_array.size() - 1; k++) {
    double frameDuration = speed * (system_observation_array[k + 1].time - system_observation_array[k].time);
    double publishDuration = ocs2::timedExecutionInSeconds([&]() { publishObservation(ros::Time::now(), system_observation_array[k]); });
    if (frameDuration > publishDuration) {
      ros::WallDuration(frameDuration - publishDuration).sleep();
    }
  }
}

void QuadrupedVisualizer::publishCartesianMarkers(ros::Time timeStamp, const contact_flag_t& contactFlags,
                                                  const feet_array_t<vector3_t>& feetPosition,
                                                  const feet_array_t<vector3_t>& feetForce) const {
  // Reserve message
  const int numberOfCartesianMarkers = 10;
  visualization_msgs::MarkerArray markerArray;
  markerArray.markers.reserve(numberOfCartesianMarkers);

  // Feet positions and Forces
  for (int i = 0; i < NUM_CONTACT_POINTS; ++i) {
    markerArray.markers.emplace_back(
        getFootMarker(feetPosition[i], contactFlags[i], feetColorMap_[i], footMarkerDiameter_, footAlphaWhenLifted_));
    markerArray.markers.emplace_back(getForceMarker(feetForce[i], feetPosition[i], contactFlags[i], ocs2::Color::green, forceScale_));
  }

  // Center of pressure
  markerArray.markers.emplace_back(getCenterOfPressureMarker(feetForce.begin(), feetForce.end(), feetPosition.begin(), contactFlags.begin(),
                                                             ocs2::Color::green, copMarkerDiameter_));

  // Support polygon
  markerArray.markers.emplace_back(getSupportPolygonMarker(feetPosition.begin(), feetPosition.end(), contactFlags.begin(),
                                                           ocs2::Color::black, supportPolygonLineWidth_));

  // Give markers an id and a frame
  ocs2::assignHeader(markerArray.markers.begin(), markerArray.markers.end(), ocs2::getHeaderMsg(frameId_, timeStamp));
  ocs2::assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

  // Publish cartesian markers (minus the CoM Pose)
  currentStatePublisher_.publish(markerArray);
}

void QuadrupedVisualizer::publishDesiredTrajectory(ros::Time timeStamp, const ocs2::TargetTrajectories& targetTrajectories) const {
  const auto& stateTrajectory = targetTrajectories.stateTrajectory;
  const auto& inputTrajectory = targetTrajectories.inputTrajectory;
  const size_t stateTrajSize = stateTrajectory.size();
  const size_t inputTrajSize = inputTrajectory.size();

  // Reserve com messages
  std::vector<geometry_msgs::Point> desiredComPositionMsg;
  desiredComPositionMsg.reserve(stateTrajSize);
  geometry_msgs::PoseArray poseArray;
  poseArray.poses.reserve(stateTrajSize);
  visualization_msgs::MarkerArray velArray;
  velArray.markers.reserve(stateTrajSize);
  visualization_msgs::MarkerArray angVelArray;
  angVelArray.markers.reserve(stateTrajSize);

  // Reserve feet messages
  feet_array_t<std::vector<geometry_msgs::Point>> desiredFeetPositionMsgs;
  feet_array_t<visualization_msgs::MarkerArray> feetVelArray;
  for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
    desiredFeetPositionMsgs[i].reserve(stateTrajSize);
    feetVelArray[i].markers.reserve(stateTrajSize);
  }
  feet_array_t<geometry_msgs::PoseArray> feetPoseArrays;
  std::for_each(feetPoseArrays.begin(), feetPoseArrays.end(), [&](geometry_msgs::PoseArray& p) { p.poses.reserve(stateTrajSize); });

  for (size_t k = 0; k < stateTrajSize; ++k) {
    const state_vector_t state = stateTrajectory[k];
    input_vector_t input;
    if (k < inputTrajSize) {
      input = inputTrajectory[k];
    } else {
      input.setZero();
    }

    // Extract elements from state
    const base_coordinate_t basePose = state.head<6>();
    const base_coordinate_t baseTwist = state.segment<6>(6);
    const auto basePositionInOrigin = getPositionInOrigin(basePose);
    const auto eulerXYZ = getOrientation(basePose);
    const Eigen::Matrix3d o_R_b = rotationMatrixBaseToOrigin<scalar_t>(eulerXYZ);
    const auto qJoints = getJointPositions(state);
    const auto qVelJoints = getJointVelocities(input);

    // Construct pose msg
    geometry_msgs::Pose pose;
    pose.position = ocs2::getPointMsg(basePositionInOrigin);
    pose.orientation = ocs2::getOrientationMsg(quaternionBaseToOrigin<double>(eulerXYZ));

    // Construct vel msg
    const vector3_t o_baseVel = o_R_b * getLinearVelocity(baseTwist);
    const vector3_t o_baseAngVel = o_R_b * getAngularVelocity(baseTwist);

    // Fill message containers
    desiredComPositionMsg.push_back(pose.position);
    poseArray.poses.push_back(std::move(pose));
    velArray.markers.emplace_back(getArrowAtPointMsg(o_baseVel / velScale_, basePositionInOrigin, ocs2::Color::blue));
    angVelArray.markers.emplace_back(getArrowAtPointMsg(o_baseAngVel / velScale_, basePositionInOrigin, ocs2::Color::green));

    for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
      const auto o_footPosition = kinematicModelPtr_->footPositionInOriginFrame(i, basePose, qJoints);
      const auto o_footVelocity = kinematicModelPtr_->footVelocityInOriginFrame(i, basePose, baseTwist, qJoints, qVelJoints);
      geometry_msgs::Pose footPose;
      footPose.position = ocs2::getPointMsg(o_footPosition);
      footPose.orientation =
          ocs2::getOrientationMsg(Eigen::Quaternion<scalar_t>(kinematicModelPtr_->footOrientationInOriginFrame(i, basePose, qJoints)));
      desiredFeetPositionMsgs[i].push_back(footPose.position);
      feetPoseArrays[i].poses.push_back(std::move(footPose));
      feetVelArray[i].markers.emplace_back(getArrowAtPointMsg(o_footVelocity / velScale_, o_footPosition, feetColorMap_[i]));
    }
  }

  // Headers
  auto comLineMsg = getLineMsg(std::move(desiredComPositionMsg), ocs2::Color::green, trajectoryLineWidth_);
  comLineMsg.header = ocs2::getHeaderMsg(frameId_, timeStamp);
  comLineMsg.id = 0;
  poseArray.header = ocs2::getHeaderMsg(frameId_, timeStamp);
  ocs2::assignHeader(feetPoseArrays.begin(), feetPoseArrays.end(), ocs2::getHeaderMsg(frameId_, timeStamp));
  ocs2::assignHeader(velArray.markers.begin(), velArray.markers.end(), ocs2::getHeaderMsg(frameId_, timeStamp));
  ocs2::assignIncreasingId(velArray.markers.begin(), velArray.markers.end());
  ocs2::assignHeader(angVelArray.markers.begin(), angVelArray.markers.end(), ocs2::getHeaderMsg(frameId_, timeStamp));
  ocs2::assignIncreasingId(angVelArray.markers.begin(), angVelArray.markers.end());

  // Publish
  costDesiredBasePositionPublisher_.publish(comLineMsg);
  costDesiredBasePosePublisher_.publish(poseArray);
  costDesiredBaseVelocityPublisher_.publish(velArray);
  costDesiredBaseAngVelocityPublisher_.publish(angVelArray);
  for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
    auto footLineMsg = getLineMsg(std::move(desiredFeetPositionMsgs[i]), feetColorMap_[i], trajectoryLineWidth_);
    footLineMsg.header = ocs2::getHeaderMsg(frameId_, timeStamp);
    footLineMsg.id = 0;
    costDesiredFeetPosePublishers_[i].publish(feetPoseArrays[i]);
    costDesiredFeetPositionPublishers_[i].publish(footLineMsg);
    ocs2::assignHeader(feetVelArray[i].markers.begin(), feetVelArray[i].markers.end(), ocs2::getHeaderMsg(frameId_, timeStamp));
    ocs2::assignIncreasingId(feetVelArray[i].markers.begin(), feetVelArray[i].markers.end());
    costDesiredFeetVelocityPublishers_[i].publish(feetVelArray[i]);
  }
}

void QuadrupedVisualizer::publishOptimizedStateTrajectory(ros::Time timeStamp, const scalar_array_t& mpcTimeTrajectory,
                                                          const vector_array_t& mpcStateTrajectory,
                                                          const ocs2::ModeSchedule& modeSchedule) const {
  if (mpcTimeTrajectory.empty() || mpcStateTrajectory.empty()) {
    return;  // Nothing to publish
  }

  // Reserve Feet msg
  feet_array_t<std::vector<geometry_msgs::Point>> feetMsgs;
  std::for_each(feetMsgs.begin(), feetMsgs.end(), [&](std::vector<geometry_msgs::Point>& v) { v.reserve(mpcStateTrajectory.size()); });
  feet_array_t<geometry_msgs::PoseArray> feetPoseMsgs;
  std::for_each(feetPoseMsgs.begin(), feetPoseMsgs.end(), [&](geometry_msgs::PoseArray& p) { p.poses.reserve(mpcStateTrajectory.size()); });

  // Reserve Com Msg
  std::vector<geometry_msgs::Point> mpcComPositionMsgs;
  mpcComPositionMsgs.reserve(mpcStateTrajectory.size());

  // Reserve Pose array
  geometry_msgs::PoseArray poseArray;
  poseArray.poses.reserve(mpcStateTrajectory.size());

  // Extract Com and Feet from state
  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const vector_t& state) {
    const state_vector_t switchedState = state.head(STATE_DIM);
    const base_coordinate_t basePose = getBasePose(switchedState);
    const joint_coordinate_t qJoints = getJointPositions(switchedState);

    // Fill com position and pose msgs
    geometry_msgs::Pose pose;
    pose.position = ocs2::getPointMsg(getPositionInOrigin(basePose));
    pose.orientation = ocs2::getOrientationMsg(quaternionBaseToOrigin<double>(getOrientation(basePose)));
    mpcComPositionMsgs.push_back(pose.position);
    poseArray.poses.push_back(std::move(pose));

    // Fill feet msgs
    for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
      const auto o_feetPosition = kinematicModelPtr_->footPositionInOriginFrame(i, basePose, qJoints);
      const auto position = ocs2::getPointMsg(o_feetPosition);
      feetMsgs[i].push_back(position);
      geometry_msgs::Pose footPose;
      footPose.position = position;
      footPose.orientation =
          ocs2::getOrientationMsg(Eigen::Quaternion<scalar_t>(kinematicModelPtr_->footOrientationInOriginFrame(i, basePose, qJoints)));
      feetPoseMsgs[i].poses.push_back(std::move(footPose));
    }
  });

  // Convert feet msgs to Array message
  visualization_msgs::MarkerArray markerArray;
  markerArray.markers.reserve(NUM_CONTACT_POINTS + 2);  // 1 trajectory per foot + 1 for the future footholds + 1 for the com trajectory
  for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
    markerArray.markers.emplace_back(getLineMsg(std::move(feetMsgs[i]), feetColorMap_[i], trajectoryLineWidth_));
    markerArray.markers.back().ns = "EE Trajectories";
  }
  markerArray.markers.emplace_back(getLineMsg(std::move(mpcComPositionMsgs), ocs2::Color::red, trajectoryLineWidth_));
  markerArray.markers.back().ns = "CoM Trajectory";

  // Future footholds
  visualization_msgs::Marker sphereList;
  sphereList.type = visualization_msgs::Marker::SPHERE_LIST;
  sphereList.scale.x = footMarkerDiameter_;
  sphereList.scale.y = footMarkerDiameter_;
  sphereList.scale.z = footMarkerDiameter_;
  sphereList.ns = "Future footholds";
  sphereList.pose.orientation = ocs2::getOrientationMsg({1., 0., 0., 0.});
  const auto& eventTimes = modeSchedule.eventTimes;
  const auto& subsystemSequence = modeSchedule.modeSequence;
  const double tStart = mpcTimeTrajectory.front();
  const double tEnd = mpcTimeTrajectory.back();
  for (int event = 0; event < eventTimes.size(); ++event) {
    if (tStart < eventTimes[event] && eventTimes[event] < tEnd) {  // Only publish future footholds within the optimized horizon
      const auto preEventContactFlags = modeNumber2StanceLeg(subsystemSequence[event]);
      const auto postEventContactFlags = modeNumber2StanceLeg(subsystemSequence[event + 1]);
      const vector_t postEventState = ocs2::LinearInterpolation::interpolate(eventTimes[event], mpcTimeTrajectory, mpcStateTrajectory);
      const state_vector_t postEventSwitchedState = postEventState.head(STATE_DIM);
      const base_coordinate_t basePose = getBasePose(postEventSwitchedState);
      const joint_coordinate_t qJoints = getJointPositions(postEventSwitchedState);

      for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
        if (!preEventContactFlags[i] && postEventContactFlags[i]) {  // If a foot lands, a marker is added at that location.
          const auto o_feetPosition = kinematicModelPtr_->footPositionInOriginFrame(i, basePose, qJoints);
          sphereList.points.emplace_back(ocs2::getPointMsg(o_feetPosition));
          sphereList.colors.push_back(getColor(feetColorMap_[i]));
        }
      }
    }
  }
  markerArray.markers.push_back(std::move(sphereList));

  // Add headers and Id
  ocs2::assignHeader(markerArray.markers.begin(), markerArray.markers.end(), ocs2::getHeaderMsg(frameId_, timeStamp));
  ocs2::assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());
  poseArray.header = ocs2::getHeaderMsg(frameId_, timeStamp);
  ocs2::assignHeader(feetPoseMsgs.begin(), feetPoseMsgs.end(), ocs2::getHeaderMsg(frameId_, timeStamp));

  stateOptimizedPublisher_.publish(markerArray);
  stateOptimizedPosePublisher_.publish(poseArray);
  for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
    stateOptimizedFeetPosePublishers_[i].publish(feetPoseMsgs[i]);
  }
}

void QuadrupedVisualizer::publishEndEffectorPoses(ros::Time timeStamp, const feet_array_t<vector3_t>& feetPositions,
                                                  const feet_array_t<Eigen::Quaternion<scalar_t>>& feetOrientations) const {
  // Feet positions and Forces
  geometry_msgs::PoseArray poseArray;
  poseArray.header = ocs2::getHeaderMsg(frameId_, timeStamp);
  for (int i = 0; i < NUM_CONTACT_POINTS; ++i) {
    geometry_msgs::Pose pose;
    pose.position = ocs2::getPointMsg(feetPositions[i]);
    pose.orientation = ocs2::getOrientationMsg(feetOrientations[i]);
    poseArray.poses.push_back(std::move(pose));
  }

  currentFeetPosesPublisher_.publish(poseArray);
}

void QuadrupedVisualizer::publishCollisionSpheres(ros::Time timeStamp, const base_coordinate_t& basePose,
                                                  const joint_coordinate_t& jointAngles) const {
  const auto collisionSpheres = kinematicModelPtr_->collisionSpheresInOriginFrame(basePose, jointAngles);

  visualization_msgs::MarkerArray markerArray;
  markerArray.markers.reserve(collisionSpheres.size());

  for (const auto& sphere : collisionSpheres) {
    markerArray.markers.emplace_back(ocs2::getSphereMsg(sphere.position, ocs2::Color::red, 2.0 * sphere.radius));
  }

  // Give markers an id and a frame
  ocs2::assignHeader(markerArray.markers.begin(), markerArray.markers.end(), ocs2::getHeaderMsg(frameId_, timeStamp));
  ocs2::assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

  currentCollisionSpheresPublisher_.publish(markerArray);
}

}  // namespace switched_model
