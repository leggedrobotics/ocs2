//
// Created by rgrandia on 13.02.19.
//

#include "ocs2_quadruped_interface/QuadrupedVisualizer.h"

#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

// URDF stuff
#include <urdf/model.h>

#include <kdl_parser/kdl_parser.hpp>

// Switched model conversions
#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"
#include "ocs2_switched_model_interface/core/Rotations.h"

namespace switched_model {

QuadrupedVisualizer::QuadrupedVisualizer(
    const kinematic_model_t& kinematicModel,
    std::vector<std::string> jointNames, std::string baseName,
    const rclcpp::Node::SharedPtr& node, scalar_t maxUpdateFrequency)
    : node_(node),
      kinematicModelPtr_(kinematicModel.clone()),
      lastTime_(std::numeric_limits<scalar_t>::lowest()),
      minPublishTimeDifference_(1.0 / maxUpdateFrequency) {
  costDesiredBasePositionPublisher_ =
      node->create_publisher<visualization_msgs::msg::Marker>(
          "/ocs2_anymal/desiredBaseTrajectory", 1);
  costDesiredBasePosePublisher_ =
      node->create_publisher<geometry_msgs::msg::PoseArray>(
          "/ocs2_anymal/desiredPoseTrajectory", 1);
  costDesiredBaseAngVelocityPublisher_ =
      node->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/ocs2_anymal/desiredAngVelTrajectory", 1);
  costDesiredBaseVelocityPublisher_ =
      node->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/ocs2_anymal/desiredVelTrajectory", 1);
  costDesiredFeetPositionPublishers_[0] =
      node->create_publisher<visualization_msgs::msg::Marker>(
          "/ocs2_anymal/desiredFeetTrajectory/LF", 1);
  costDesiredFeetPositionPublishers_[1] =
      node->create_publisher<visualization_msgs::msg::Marker>(
          "/ocs2_anymal/desiredFeetTrajectory/RF", 1);
  costDesiredFeetPositionPublishers_[2] =
      node->create_publisher<visualization_msgs::msg::Marker>(
          "/ocs2_anymal/desiredFeetTrajectory/LH", 1);
  costDesiredFeetPositionPublishers_[3] =
      node->create_publisher<visualization_msgs::msg::Marker>(
          "/ocs2_anymal/desiredFeetTrajectory/RH", 1);
  costDesiredFeetVelocityPublishers_[0] =
      node->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/ocs2_anymal/desiredFeetVelTrajectory/LF", 1);
  costDesiredFeetVelocityPublishers_[1] =
      node->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/ocs2_anymal/desiredFeetVelTrajectory/RF", 1);
  costDesiredFeetVelocityPublishers_[2] =
      node->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/ocs2_anymal/desiredFeetVelTrajectory/LH", 1);
  costDesiredFeetVelocityPublishers_[3] =
      node->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/ocs2_anymal/desiredFeetVelTrajectory/RH", 1);
  stateOptimizedPublisher_ =
      node->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/ocs2_anymal/optimizedStateTrajectory", 1);
  stateOptimizedPosePublisher_ =
      node->create_publisher<geometry_msgs::msg::PoseArray>(
          "/ocs2_anymal/optimizedPoseTrajectory", 1);
  currentStatePublisher_ =
      node->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/ocs2_anymal/currentState", 1);
  currentFeetPosesPublisher_ =
      node->create_publisher<geometry_msgs::msg::PoseArray>(
          "/ocs2_anymal/currentFeetPoses", 1);
  currentCollisionSpheresPublisher_ =
      node->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/ocs2_anymal/currentCollisionSpheres", 1);

  quadrupedTfPublisher_.launchNode(node, "ocs2_anymal_description",
                                   std::move(jointNames), std::move(baseName));
}

void QuadrupedVisualizer::update(const ocs2::SystemObservation& observation,
                                 const ocs2::PrimalSolution& primalSolution,
                                 const ocs2::CommandData& command) {
  if (observation.time - lastTime_ > minPublishTimeDifference_) {
    const auto timeStamp = node_->get_clock()->now();
    publishObservation(timeStamp, observation);
    publishDesiredTrajectory(timeStamp, command.mpcTargetTrajectories_);
    publishOptimizedStateTrajectory(timeStamp, primalSolution.timeTrajectory_,
                                    primalSolution.stateTrajectory_,
                                    primalSolution.modeSchedule_);
    lastTime_ = observation.time;
  }
}

void QuadrupedVisualizer::publishObservation(
    rclcpp::Time timeStamp, const ocs2::SystemObservation& observation) {
  // Extract components from state
  const state_vector_t switchedState = observation.state.head(STATE_DIM);
  const base_coordinate_t basePose = getBasePose(switchedState);
  const auto o_basePosition = getPositionInOrigin(basePose);
  const Eigen::Matrix3d o_R_b =
      rotationMatrixBaseToOrigin<scalar_t>(getOrientation(basePose));
  const joint_coordinate_t qJoints =
      getJointPositions(state_vector_t(observation.state));

  // Compute cartesian state and inputs
  feet_array_t<vector3_t> feetPosition;
  feet_array_t<vector3_t> feetForce;
  feet_array_t<Eigen::Quaternion<scalar_t>> feetOrientations;
  for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
    feetPosition[i] = o_basePosition;
    feetPosition[i].noalias() +=
        o_R_b * kinematicModelPtr_->positionBaseToFootInBaseFrame(i, qJoints);

    feetForce[i] = o_R_b * observation.input.segment<3>(3 * i);

    const matrix3_t footOrientationInOriginFrame =
        o_R_b * kinematicModelPtr_->footOrientationInBaseFrame(i, qJoints);
    feetOrientations[i] =
        Eigen::Quaternion<scalar_t>(footOrientationInOriginFrame);
  }

  // Publish
  quadrupedTfPublisher_.publish(timeStamp, basePose, qJoints, frameId_);
  publishCartesianMarkers(timeStamp, modeNumber2StanceLeg(observation.mode),
                          feetPosition, feetForce);
  publishEndEffectorPoses(timeStamp, feetPosition, feetOrientations);
  publishCollisionSpheres(timeStamp, basePose, qJoints);
}

void QuadrupedVisualizer::publishTrajectory(
    const std::vector<ocs2::SystemObservation>& system_observation_array,
    double speed) {
  for (size_t k = 0; k < system_observation_array.size() - 1; k++) {
    double frameDuration = speed * (system_observation_array[k + 1].time -
                                    system_observation_array[k].time);
    double publishDuration = ocs2::timedExecutionInSeconds([&]() {
      publishObservation(node_->get_clock()->now(),
                         system_observation_array[k]);
    });
    if (frameDuration > publishDuration) {
      const rclcpp::Duration duration =
          rclcpp::Duration::from_seconds(frameDuration - publishDuration);
      rclcpp::sleep_for((std::chrono::nanoseconds(duration.nanoseconds())));
    }
  }
}

void QuadrupedVisualizer::publishCartesianMarkers(
    rclcpp::Time timeStamp, const contact_flag_t& contactFlags,
    const feet_array_t<vector3_t>& feetPosition,
    const feet_array_t<vector3_t>& feetForce) const {
  // Reserve message
  const int numberOfCartesianMarkers = 10;
  visualization_msgs::msg::MarkerArray markerArray;
  markerArray.markers.reserve(numberOfCartesianMarkers);

  // Feet positions and Forces
  for (int i = 0; i < NUM_CONTACT_POINTS; ++i) {
    markerArray.markers.emplace_back(
        getFootMarker(feetPosition[i], contactFlags[i], feetColorMap_[i],
                      footMarkerDiameter_, footAlphaWhenLifted_));
    markerArray.markers.emplace_back(
        getForceMarker(feetForce[i], feetPosition[i], contactFlags[i],
                       ocs2::Color::green, forceScale_));
  }

  // Center of pressure
  markerArray.markers.emplace_back(getCenterOfPressureMarker(
      feetForce.begin(), feetForce.end(), feetPosition.begin(),
      contactFlags.begin(), ocs2::Color::green, copMarkerDiameter_));

  // Support polygon
  markerArray.markers.emplace_back(getSupportPolygonMarker(
      feetPosition.begin(), feetPosition.end(), contactFlags.begin(),
      ocs2::Color::black, supportPolygonLineWidth_));

  // Give markers an id and a frame
  ocs2::assignHeader(markerArray.markers.begin(), markerArray.markers.end(),
                     ocs2::getHeaderMsg(frameId_, timeStamp));
  ocs2::assignIncreasingId(markerArray.markers.begin(),
                           markerArray.markers.end());

  // Publish cartesian markers (minus the CoM Pose)
  currentStatePublisher_->publish(markerArray);
}

void QuadrupedVisualizer::publishDesiredTrajectory(
    rclcpp::Time timeStamp,
    const ocs2::TargetTrajectories& targetTrajectories) const {
  const auto& stateTrajectory = targetTrajectories.stateTrajectory;
  const auto& inputTrajectory = targetTrajectories.inputTrajectory;
  const size_t stateTrajSize = stateTrajectory.size();
  const size_t inputTrajSize = inputTrajectory.size();

  // Reserve Base messages
  std::vector<geometry_msgs::msg::Point> desiredBasePositionMsg;
  desiredBasePositionMsg.reserve(stateTrajSize);
  geometry_msgs::msg::PoseArray poseArray;
  poseArray.poses.reserve(stateTrajSize);
  visualization_msgs::msg::MarkerArray velArray;
  velArray.markers.reserve(stateTrajSize);
  visualization_msgs::msg::MarkerArray angVelArray;
  angVelArray.markers.reserve(stateTrajSize);

  // Reserve feet messages
  feet_array_t<std::vector<geometry_msgs::msg::Point>> desiredFeetPositionMsgs;
  feet_array_t<visualization_msgs::msg::MarkerArray> feetVelArray;
  for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
    desiredFeetPositionMsgs[i].reserve(stateTrajSize);
    feetVelArray[i].markers.reserve(stateTrajSize);
  }

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
    const auto o_basePosition = getPositionInOrigin(basePose);
    const auto eulerXYZ = getOrientation(basePose);
    const Eigen::Matrix3d o_R_b =
        rotationMatrixBaseToOrigin<scalar_t>(eulerXYZ);
    const auto qJoints = getJointPositions(state);
    const auto qVelJoints = getJointVelocities(input);

    // Construct pose msg
    geometry_msgs::msg::Pose pose;
    pose.position = ocs2::getPointMsg(o_basePosition);
    pose.orientation =
        ocs2::getOrientationMsg(quaternionBaseToOrigin<double>(eulerXYZ));

    // Construct vel msg
    const vector3_t o_baseVel = o_R_b * getLinearVelocity(baseTwist);
    const vector3_t o_baseAngVel = o_R_b * getAngularVelocity(baseTwist);

    // Fill message containers
    desiredBasePositionMsg.push_back(pose.position);
    poseArray.poses.push_back(std::move(pose));
    velArray.markers.emplace_back(getArrowAtPointMsg(
        o_baseVel / velScale_, o_basePosition, ocs2::Color::blue));
    angVelArray.markers.emplace_back(getArrowAtPointMsg(
        o_baseAngVel / velScale_, o_basePosition, ocs2::Color::green));

    for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
      const vector3_t b_baseToFoot =
          kinematicModelPtr_->positionBaseToFootInBaseFrame(i, qJoints);
      auto o_footPosition = o_basePosition;
      o_footPosition.noalias() += o_R_b * b_baseToFoot;
      const vector3_t b_footVelocity =
          kinematicModelPtr_->footVelocityRelativeToBaseInBaseFrame(
              i, qJoints, qVelJoints) +
          getLinearVelocity(baseTwist) +
          getAngularVelocity(baseTwist).cross(b_baseToFoot);
      const vector3_t o_footVelocity = o_R_b * b_footVelocity;
      desiredFeetPositionMsgs[i].push_back(ocs2::getPointMsg(o_footPosition));
      feetVelArray[i].markers.emplace_back(getArrowAtPointMsg(
          o_footVelocity / velScale_, o_footPosition, feetColorMap_[i]));
    }
  }

  // Headers
  auto baseLineMsg = getLineMsg(std::move(desiredBasePositionMsg),
                                ocs2::Color::green, trajectoryLineWidth_);
  baseLineMsg.header = ocs2::getHeaderMsg(frameId_, timeStamp);
  baseLineMsg.id = 0;
  poseArray.header = ocs2::getHeaderMsg(frameId_, timeStamp);

  ocs2::assignHeader(velArray.markers.begin(), velArray.markers.end(),
                     ocs2::getHeaderMsg(frameId_, timeStamp));
  ocs2::assignIncreasingId(velArray.markers.begin(), velArray.markers.end());
  ocs2::assignHeader(angVelArray.markers.begin(), angVelArray.markers.end(),
                     ocs2::getHeaderMsg(frameId_, timeStamp));
  ocs2::assignIncreasingId(angVelArray.markers.begin(),
                           angVelArray.markers.end());

  // Publish
  costDesiredBasePositionPublisher_->publish(baseLineMsg);
  costDesiredBasePosePublisher_->publish(poseArray);
  costDesiredBaseVelocityPublisher_->publish(velArray);
  costDesiredBaseAngVelocityPublisher_->publish(angVelArray);
  for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
    auto footLineMsg = getLineMsg(std::move(desiredFeetPositionMsgs[i]),
                                  feetColorMap_[i], trajectoryLineWidth_);
    footLineMsg.header = ocs2::getHeaderMsg(frameId_, timeStamp);
    footLineMsg.id = 0;
    costDesiredFeetPositionPublishers_[i]->publish(footLineMsg);
    ocs2::assignHeader(feetVelArray[i].markers.begin(),
                       feetVelArray[i].markers.end(),
                       ocs2::getHeaderMsg(frameId_, timeStamp));
    ocs2::assignIncreasingId(feetVelArray[i].markers.begin(),
                             feetVelArray[i].markers.end());
    costDesiredFeetVelocityPublishers_[i]->publish(feetVelArray[i]);
  }
}

void QuadrupedVisualizer::publishOptimizedStateTrajectory(
    rclcpp::Time timeStamp, const scalar_array_t& mpcTimeTrajectory,
    const vector_array_t& mpcStateTrajectory,
    const ocs2::ModeSchedule& modeSchedule) const {
  if (mpcTimeTrajectory.empty() || mpcStateTrajectory.empty()) {
    return;  // Nothing to publish
  }

  // Reserve Feet msg
  feet_array_t<std::vector<geometry_msgs::msg::Point>> feetMsgs;
  std::for_each(feetMsgs.begin(), feetMsgs.end(),
                [&](std::vector<geometry_msgs::msg::Point>& v) {
                  v.reserve(mpcStateTrajectory.size());
                });

  // Reserve Base Msg
  std::vector<geometry_msgs::msg::Point> mpcBasePositionMsgs;
  mpcBasePositionMsgs.reserve(mpcStateTrajectory.size());

  // Reserve Pose array
  geometry_msgs::msg::PoseArray poseArray;
  poseArray.poses.reserve(mpcStateTrajectory.size());

  // Extract Base and Feet from state
  std::for_each(
      mpcStateTrajectory.begin(), mpcStateTrajectory.end(),
      [&](const vector_t& state) {
        const state_vector_t switchedState = state.head(STATE_DIM);
        const base_coordinate_t basePose = getBasePose(switchedState);
        const vector3_t o_basePosition = getPositionInOrigin(basePose);
        const vector3_t eulerXYZ = getOrientation(basePose);
        const matrix3_t o_R_b = rotationMatrixBaseToOrigin(eulerXYZ);
        const joint_coordinate_t qJoints = getJointPositions(switchedState);

        // Fill Base position and pose msgs
        geometry_msgs::msg::Pose pose;
        pose.position = ocs2::getPointMsg(o_basePosition);
        pose.orientation =
            ocs2::getOrientationMsg(quaternionBaseToOrigin<double>(eulerXYZ));
        mpcBasePositionMsgs.push_back(pose.position);
        poseArray.poses.push_back(std::move(pose));

        // Fill feet msgs
        for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
          auto o_footPosition = o_basePosition;
          o_footPosition.noalias() +=
              o_R_b *
              kinematicModelPtr_->positionBaseToFootInBaseFrame(i, qJoints);
          feetMsgs[i].push_back(ocs2::getPointMsg(o_footPosition));
        }
      });

  // Convert feet msgs to Array message
  visualization_msgs::msg::MarkerArray markerArray;
  markerArray.markers.reserve(NUM_CONTACT_POINTS +
                              2);  // 1 trajectory per foot + 1 for the future
                                   // footholds + 1 for the Base trajectory
  for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
    markerArray.markers.emplace_back(getLineMsg(
        std::move(feetMsgs[i]), feetColorMap_[i], trajectoryLineWidth_));
    markerArray.markers.back().ns = "EE Trajectories";
  }
  markerArray.markers.emplace_back(getLineMsg(
      std::move(mpcBasePositionMsgs), ocs2::Color::red, trajectoryLineWidth_));
  markerArray.markers.back().ns = "Base Trajectory";

  // Future footholds
  visualization_msgs::msg::Marker sphereList;
  sphereList.type = visualization_msgs::msg::Marker::SPHERE_LIST;
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
    if (tStart < eventTimes[event] &&
        eventTimes[event] < tEnd) {  // Only publish future footholds within the
                                     // optimized horizon
      const auto preEventContactFlags =
          modeNumber2StanceLeg(subsystemSequence[event]);
      const auto postEventContactFlags =
          modeNumber2StanceLeg(subsystemSequence[event + 1]);
      const vector_t postEventState = ocs2::LinearInterpolation::interpolate(
          eventTimes[event], mpcTimeTrajectory, mpcStateTrajectory);
      const state_vector_t postEventSwitchedState =
          postEventState.head(STATE_DIM);
      const base_coordinate_t basePose = getBasePose(postEventSwitchedState);
      const joint_coordinate_t qJoints =
          getJointPositions(postEventSwitchedState);

      for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
        if (!preEventContactFlags[i] &&
            postEventContactFlags[i]) {  // If a foot lands, a marker is added
                                         // at that location.
          const auto o_feetPosition =
              kinematicModelPtr_->footPositionInOriginFrame(i, basePose,
                                                            qJoints);
          sphereList.points.emplace_back(ocs2::getPointMsg(o_feetPosition));
          sphereList.colors.push_back(getColor(feetColorMap_[i]));
        }
      }
    }
  }
  markerArray.markers.push_back(std::move(sphereList));

  // Add headers and Id
  ocs2::assignHeader(markerArray.markers.begin(), markerArray.markers.end(),
                     ocs2::getHeaderMsg(frameId_, timeStamp));
  ocs2::assignIncreasingId(markerArray.markers.begin(),
                           markerArray.markers.end());
  poseArray.header = ocs2::getHeaderMsg(frameId_, timeStamp);

  stateOptimizedPublisher_->publish(markerArray);
  stateOptimizedPosePublisher_->publish(poseArray);
}

void QuadrupedVisualizer::publishEndEffectorPoses(
    rclcpp::Time timeStamp, const feet_array_t<vector3_t>& feetPositions,
    const feet_array_t<Eigen::Quaternion<scalar_t>>& feetOrientations) const {
  // Feet positions and Forces
  geometry_msgs::msg::PoseArray poseArray;
  poseArray.header = ocs2::getHeaderMsg(frameId_, timeStamp);
  for (int i = 0; i < NUM_CONTACT_POINTS; ++i) {
    geometry_msgs::msg::Pose pose;
    pose.position = ocs2::getPointMsg(feetPositions[i]);
    pose.orientation = ocs2::getOrientationMsg(feetOrientations[i]);
    poseArray.poses.push_back(std::move(pose));
  }

  currentFeetPosesPublisher_->publish(poseArray);
}

void QuadrupedVisualizer::publishCollisionSpheres(
    rclcpp::Time timeStamp, const base_coordinate_t& basePose,
    const joint_coordinate_t& jointAngles) const {
  const auto collisionSpheres =
      kinematicModelPtr_->collisionSpheresInOriginFrame(basePose, jointAngles);

  visualization_msgs::msg::MarkerArray markerArray;
  markerArray.markers.reserve(collisionSpheres.size());

  for (const auto& sphere : collisionSpheres) {
    markerArray.markers.emplace_back(ocs2::getSphereMsg(
        sphere.position, ocs2::Color::red, 2.0 * sphere.radius));
  }

  // Give markers an id and a frame
  ocs2::assignHeader(markerArray.markers.begin(), markerArray.markers.end(),
                     ocs2::getHeaderMsg(frameId_, timeStamp));
  ocs2::assignIncreasingId(markerArray.markers.begin(),
                           markerArray.markers.end());

  currentCollisionSpheresPublisher_->publish(markerArray);
}

}  // namespace switched_model
