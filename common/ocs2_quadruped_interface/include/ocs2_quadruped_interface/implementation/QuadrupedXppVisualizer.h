//
// Created by rgrandia on 13.02.19.
//
#include "ocs2_quadruped_interface/QuadrupedXppVisualizer.h"

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>

#include <ocs2_core/Dimensions.h>
#include "ocs2_switched_model_interface/core/Rotations.h"
#include <ocs2_msgs/mpc_target_trajectories.h>
#include "ocs2_comm_interfaces/ocs2_ros_interfaces/common/RosMsgConversions.h"

namespace switched_model {

  template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
    inline void QuadrupedXppVisualizer<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::
    setRobotStateCartesianEEValues(
        const vector_3d_array_t& o_feetPosition,
        const vector_3d_array_t& o_feetVelocity,
        const vector_3d_array_t& o_feetAcceleration,
        const vector_3d_array_t& o_feetForce,
        xpp_msgs::RobotStateCartesian& rstcm)
    {
      rstcm.ee_motion.resize(NUM_CONTACT_POINTS);
      rstcm.ee_forces.resize(NUM_CONTACT_POINTS);
      rstcm.ee_contact.resize(NUM_CONTACT_POINTS);
      for (size_t ee = 0; ee < NUM_CONTACT_POINTS; ++ee) {
        tf::pointEigenToMsg(o_feetPosition[ee],      rstcm.ee_motion[ee].pos);
        tf::vectorEigenToMsg(o_feetVelocity[ee],     rstcm.ee_motion[ee].vel);
        tf::vectorEigenToMsg(o_feetAcceleration[ee], rstcm.ee_motion[ee].acc);
        tf::vectorEigenToMsg(o_feetForce[ee],        rstcm.ee_forces[ee]);
        /* Check if the forces are 0 */
        rstcm.ee_contact[ee] = !o_feetForce[ee].isZero(0);
      }
    }

  template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
    inline void QuadrupedXppVisualizer<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::
    setRobotStateCartesianEEValues(
        const state_vector_t& state,
        const input_vector_t& input,
        xpp_msgs::RobotStateCartesian& rstcm)
    {
      vector_3d_array_t o_feetPosition, o_feetVelocity, o_feetAcceleration, o_feetForce;
      computeFeetState(state, input, o_feetPosition, o_feetVelocity, o_feetForce);
      for (size_t ee=0; ee < NUM_CONTACT_POINTS; ++ee) { o_feetAcceleration[ee].setZero(); /* TODO(oharley): set to 0 since we don't have these, or just leave? */ }
      setRobotStateCartesianEEValues(
          o_feetPosition,
          o_feetVelocity,
          o_feetAcceleration,
          o_feetForce,
          rstcm);
    }

  template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
    void QuadrupedXppVisualizer<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::targetTrajectoriesCallback(
        const ocs2_msgs::mpc_target_trajectories& msg)
    {
      cost_desired_trajectories_t cdt;
      ocs2::RosMsgConversions<STATE_DIM, INPUT_DIM>::readTargetTrajectoriesMsg(msg, cdt);
      scalar_t timeNow = ros::Time::now().toSec();
      publishXppCostsVisualizer(timeNow, cdt);
      publishDesiredTrajectory(timeNow, cdt);
    }

  template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
    void QuadrupedXppVisualizer<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::launchVisualizerNode(int argc, char** argv) {
      ros::init(argc, argv, robotName_ + "_visualization_node");
      signal(SIGINT, QuadrupedXppVisualizer::sigintHandler);

      ros::NodeHandle n;
      n.setCallbackQueue(&visualizerQueue_);

      visualizationPublisher_           = visualizers::xppStateDesTopicMap.advertise(n, 1);
      visualizationJointPublisher_      = visualizers::xppJointDesTopicMap.advertise(n, 1);
      costsVisualizationPublisher_      = visualizers::xppStateTrajTopicMap.advertise(n, 1, true);
      costsVisualizationJointPublisher_ = visualizers::xppJointTrajTopicMap.advertise(n, 1, true);
      // costsVisualizationPublisher_      = visualizers::xppStateTrajTopicMap.advertise(n, 1);
      // costsVisualizationJointPublisher_ = visualizers::xppJointTrajTopicMap.advertise(n, 1);

      comTracePublisher_     = visualizers::comTraceTopicMap.advertise(n, 100);
      feetTracePublisher_    = visualizers::feetTraceTopicMap.advertise(n, 100);
      poseTrajPublisher_     = visualizers::posesTargetTopicMap.advertise(n, 100);

      poseMPCPublisher_      = visualizers::posesMPCTopicMap.advertise(n, 100);
      comMPCTracePublisher_  = visualizers::comMPCTraceTopicMap.advertise(n, 100);
      feetMPCTracePublisher_ = visualizers::feetMPCTraceTopicMap.advertise(n, 100);


      ROS_INFO_STREAM("Waiting for visualization subscriber ...");
      while (ros::ok() && visualizationPublisher_.getNumSubscribers() == 0) {
        ros::Rate(100).sleep();
      }
      ROS_INFO_STREAM("Visualization subscriber is connected.");

      targetTrajectoriesSubscriber_ = n.subscribe( robotName_ + "_mpc_target", 1, &quadruped_xpp_visualizer_t::targetTrajectoriesCallback, this);

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

      publishXppVisualizer(observation.time(), rbdState.template head<6>(), rbdState.template segment<6>(18), rbdState.template segment<12>(6),
          o_feetPositionRef, o_feetVelocityRef, o_feetAccelerationRef, o_feetForceRef);
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

  // template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
  //   void QuadrupedXppVisualizer<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::publishXppCostsVisualizer(const scalar_t& publishTime)
  //   {
  //     const auto costDesiredTrajectories = ocs2QuadrupedInterfacePtr_->getMpc().getSolverPtr()->getCostTrajectories();
  //     publishXppCostsVisualizer(publishTime, costDesiredTrajectories);
  //   }

  template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
    void QuadrupedXppVisualizer<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::
    publishXppCostsVisualizer(const scalar_t& publishTime, const cost_desired_trajectories_t& costDesiredTrajectories)
    {
      // auto costDesiredTrajectories = ocs2QuadrupedInterfacePtr_.getCostDesiredTrajectories();
      auto N = costDesiredTrajectories.desiredStateTrajectory().size();
      std::cout << "Publishing Xpp Costs trajectories. N: "  << N <<  std::endl;
      if (!N){return;}

      for (int i = 0; i < N; ++i) {
        auto& state = costDesiredTrajectories.desiredStateTrajectory()[i];
        auto time = costDesiredTrajectories.desiredTimeTrajectory()[i];
        // construct the message
        xpp_msgs::RobotStateCartesian robotStateCartesianMsg;

        const Eigen::Quaternion<scalar_t> q_world_base = quaternionBaseToOrigin<scalar_t>(state.template head<3>());
        tf::quaternionEigenToMsg(q_world_base, robotStateCartesianMsg.base.pose.orientation);
        tf::pointEigenToMsg(state.template segment<3>(3), robotStateCartesianMsg.base.pose.position);
        tf::vectorEigenToMsg(state.template segment<3>(6), robotStateCartesianMsg.base.twist.angular);
        tf::vectorEigenToMsg(state.template segment<3>(9), robotStateCartesianMsg.base.twist.linear);

        robotStateCartesianMsg.time_from_start = ros::Duration(time);
        auto& input = costDesiredTrajectories.desiredStateTrajectory()[i];
        setRobotStateCartesianEEValues(state, input, robotStateCartesianMsg);

        // if (save_rosbag_) {
        //   const auto stamp = ros::Time(startTime_.toSec() + time);
        //   try {
        //     bag_.write("xpp/state_des", stamp, robotStateCartesianMsg);
        //   } catch (const rosbag::BagException& err) {
        //     std::cerr << "Error writing rosbag message: " << err.what() << std::endl;
        //   }
        //
        //   robotStateCartesianTrajectoryMsg_.points.push_back(robotStateCartesianMsg);
        // }

        // Joint space message
        //TODO(Oharley)
        xpp_msgs::RobotStateJoint robotStateJointMsg;
        robotStateJointMsg.time_from_start = robotStateCartesianMsg.time_from_start;
        robotStateJointMsg.base            = robotStateCartesianMsg.base;
        robotStateJointMsg.ee_contact      = robotStateCartesianMsg.ee_contact;

        const auto jointAngles = state.template segment<JOINT_COORDINATE_SIZE>(12);
        robotStateJointMsg.joint_state.position = std::vector<scalar_t>(JOINT_COORDINATE_SIZE);
        for (auto j=0;j<JOINT_COORDINATE_SIZE; ++j){
          robotStateJointMsg.joint_state.position[j] = jointAngles[j];
        }
        std::cout << "\nJointmsg @ " << robotStateJointMsg.time_from_start << " : "<< jointAngles;
        // Attention: Not filling joint velocities or torques

        // Publish
        costsVisualizationPublisher_.publish(robotStateCartesianMsg);
        costsVisualizationJointPublisher_.publish(robotStateJointMsg);
      }
    }

  template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
    void QuadrupedXppVisualizer<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::publishXppVisualizer(
        const scalar_t time, const base_coordinate_t& basePose, const base_coordinate_t& baseLocalVelocities,
        const joint_coordinate_t& jointAngles, const vector_3d_array_t& feetPosition, const vector_3d_array_t& feetVelocity,
        const vector_3d_array_t& feetAcceleration, const vector_3d_array_t& feetForce) {
      const scalar_t minTimeDifference = 10e-3;

      static scalar_t lastTime = 0.0;
      if (time - lastTime < minTimeDifference) return;

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

      setRobotStateCartesianEEValues(feetPosition, feetVelocity, feetAcceleration, feetForce, robotStateCartesianMsg);

      if (save_rosbag_) {
        const auto stamp = ros::Time(startTime_.toSec() + time);
        try {
          bag_.write("xpp/state_des", stamp, robotStateCartesianMsg);
        } catch (const rosbag::BagException& err) {
          std::cerr << "Error writing rosbag message: " << err.what() << std::endl;
        }

        robotStateCartesianTrajectoryMsg_.points.push_back(robotStateCartesianMsg);
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
      base_coordinate_t comPose = state.template head<6>();
      base_coordinate_t comLocalVelocities = state.template segment<6>(6);
      joint_coordinate_t qJoints = state.template segment<JOINT_COORD_SIZE>(12);
      joint_coordinate_t dqJoints = input.template segment<JOINT_COORD_SIZE>(12);

      base_coordinate_t basePose = ocs2QuadrupedInterfacePtr_->getComModel().calculateBasePose(comPose);
      base_coordinate_t baseLocalVelocities = ocs2QuadrupedInterfacePtr_->getComModel().calculateBaseLocalVelocities(comLocalVelocities);

      Eigen::Matrix3d o_R_b = rotationMatrixBaseToOrigin<scalar_t>(state.template head<3>());

      for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
        o_feetPosition[i] = ocs2QuadrupedInterfacePtr_->getKinematicModel().footPositionInOriginFrame(i, basePose, qJoints);
        o_feetVelocity[i] = ocs2QuadrupedInterfacePtr_->getKinematicModel().footVelocityInOriginFrame(i, basePose, baseLocalVelocities, qJoints, dqJoints);
        o_contactForces[i] = o_R_b * input.template segment<3>(3 * i);
      }
    }

  template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
    void QuadrupedXppVisualizer<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::sigintHandler(int sig) {
      ROS_INFO_STREAM("Shutting MRT node.");
      ::ros::shutdown();
    }

  // template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
  //   void QuadrupedXppVisualizer<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::publishDesiredTrajectory(
  //       scalar_t startTime) { publishDesiredTrajectory( startTime, ocs2QuadrupedInterfacePtr_->getMpc().getSolverPtr()->getCostTrajectories()); }

  template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
    void QuadrupedXppVisualizer<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::publishDesiredTrajectory(
        scalar_t startTime, const cost_desired_trajectories_t& costDesiredTrajectories) {

      // Set up state interpolator
      auto& timeTrajectory = costDesiredTrajectories.desiredTimeTrajectory();
      auto& stateTrajectory = costDesiredTrajectories.desiredStateTrajectory();
      // ocs2::EigenLinearInterpolation<typename cost_desired_trajectories_t::dynamic_vector_t> stateFunc(&timeTrajectory, &stateTrajectory);
      auto N = timeTrajectory.size();
      std::cout << "Publishing desired cost trajectories. N:"  << N << std::endl;

      if (!N){return;}
      auto endTime = timeTrajectory.back();


      ros::Duration displayTime = ros::Duration(timeTrajectory.front(), timeTrajectory.back())*5;

      // Prepare Messages
      decltype(visualizers::comTraceTopicMap)::msg_t comMarker;
      decltype(visualizers::feetTraceTopicMap)::msg_t footMarkerMsg;
      decltype(visualizers::posesTargetTopicMap)::msg_t poseArray;

      // Message header
      comMarker.header.frame_id = poseArray.header.frame_id = "world";
      comMarker.header.stamp  = poseArray.header.stamp = ros::Time(startTime_.toSec() + startTime); //TODO(oharley)
      // poseArray.header.stamp = ros::Time(endTime);

      comMarker.header.frame_id = "world";
      comMarker.id = 0;
      comMarker.type = visualization_msgs::Marker::LINE_STRIP;
      comMarker.lifetime = displayTime;
      comMarker.frame_locked = true;
      comMarker.scale.x = 0.005;  // used for line width
      // comMarker.color.r = 0.7;
      // comMarker.color.a = 1.0;

      // Foot Array message headers
      for (size_t ee = 0; ee < NUM_CONTACT_POINTS; ++ee) {
        visualization_msgs::Marker footMsg;
        footMsg.header.frame_id = "world";
        footMsg.header.stamp = ros::Time(startTime_.toSec() + startTime);
        footMsg.id = ee + 1;
        footMsg.type = visualization_msgs::Marker::LINE_STRIP;
        footMsg.lifetime =  displayTime;
        footMsg.frame_locked = true;
        footMsg.scale.x = 0.005;  // used for line width
        // footMsg.color.b = 0.7;
        // footMsg.color.a = 1.0;
        footMarkerMsg.markers.push_back(footMsg);
      }

      auto& inputTrajectory = costDesiredTrajectories.desiredStateTrajectory();
      vector_3d_array_t o_feetPosition, o_feetVelocity, o_feetForce;
      // Evaluation times
      // double dt = 0.1;
      // double t = std::min(startTime, endTime);

      // for (; t < endTime; t=std::min(t+dt, endTime)){
      // geometry_msgs::Point comPosition;
      //   Eigen::VectorXd state;
      //   stateFunc.interpolate(t, state);
      //   // geometry_msgs::Point comPosition;
      //   // tf::pointEigenToMsg(state.template segment<3>(3), comPosition);
      //   comPosition.x = state[3];
      //   comPosition.y = state[4];
      //   comPosition.z = state[5];
      //
      //   computeFeetState(state, input_vector_t::Zero(), o_feetPosition, o_feetVelocity, o_feetForce);
      //   comMarker.points.push_back(comPosition);
      //   for (int ee = 0; ee < NUM_CONTACT_POINTS; ++ee) {
      //     geometry_msgs::Point footPosition;
      //     tf::pointEigenToMsg(o_feetPosition[ee], footPosition);
      //     footMarkerMsg.markers[ee].points.push_back(footPosition);
      //   }
      // }

      std_msgs::ColorRGBA color;
      color.b=0.0;
      color.r=0.0;
      color.g=1.0;
      color.a=0.7;
      const auto trajectoryPartitions = timeTrajectory.size();
      const auto fracOfTrajectory = 1.0/trajectoryPartitions;
      for (auto i=0; i<timeTrajectory.size(); ++i) {
        geometry_msgs::Point comPosition;
        tf::pointEigenToMsg(stateTrajectory[i].template segment<3>(3), comPosition);
        comMarker.points.push_back(comPosition);

        std_msgs::ColorRGBA fadeColor(color);
        fadeColor.b  += 1.0*fracOfTrajectory*i;
        fadeColor.g  -= 1.0*fracOfTrajectory*i;
        comMarker.colors.push_back(fadeColor);

        computeFeetState(stateTrajectory[i],
            inputTrajectory[i],
            o_feetPosition,
            o_feetVelocity,
            o_feetForce);

        for (auto ee = 0; ee < NUM_CONTACT_POINTS; ++ee) {
          geometry_msgs::Point footPosition;
          tf::pointEigenToMsg(o_feetPosition[ee], footPosition);
          footMarkerMsg.markers[ee].points.push_back(footPosition);
          footMarkerMsg.markers[ee].colors.push_back(fadeColor);
        }

        geometry_msgs::Pose poseMsg;
        geometry_msgs::Quaternion qMsg;
        poseMsg.position = comPosition;
        const auto q_world_base = quaternionBaseToOrigin<scalar_t>(stateTrajectory[i].template head<3>());
        tf::quaternionEigenToMsg(q_world_base, poseMsg.orientation);
        poseArray.poses.push_back(poseMsg);
      }

      comTracePublisher_.publish(comMarker);
      feetTracePublisher_.publish(footMarkerMsg);
      poseTrajPublisher_.publish(poseArray);
    }

  template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
    void QuadrupedXppVisualizer<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::publishOptimizedStateTrajectory(
        const scalar_array_t& mpcTimeTrajectory, const state_vector_array_t& mpcStateTrajectory) {

      decltype(visualizers::comMPCTraceTopicMap)::msg_t comMarker;
      decltype(visualizers::feetMPCTraceTopicMap)::msg_t footMarkerMsg;
      decltype(visualizers::posesMPCTopicMap)::msg_t poseArray;

      static size_t k=0;
      const auto max_ids=1000;
      // k = (k<max_ids) ? k+1 : 0 
      const auto display_lifetime=5;

      // Message header
      poseArray.header.frame_id = "world";
      comMarker.header.frame_id = "world";
      comMarker.id = k;
      comMarker.type = visualization_msgs::Marker::LINE_STRIP;
      comMarker.frame_locked = true;
      comMarker.lifetime = ros::Duration(display_lifetime);
      comMarker.scale.x = 0.005;  // used for line width
      comMarker.color.r = 1.0;
      comMarker.color.a = 1.0;

      // Foot Array message headers
      for (size_t ee = 0; ee < NUM_CONTACT_POINTS; ++ee) {
        visualization_msgs::Marker footMsg;
        footMsg.header.frame_id = "world";
        footMsg.id = ee + max_ids + 1 ;
        footMsg.type = visualization_msgs::Marker::LINE_STRIP;
        footMsg.lifetime = ros::Duration(display_lifetime);
        footMsg.frame_locked = true;
        footMsg.scale.x = 0.005;  // used for line width
        footMsg.color.b = 1.0;
        footMsg.color.a = 1.0;
        footMarkerMsg.markers.push_back(footMsg);
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

      state_vector_t state;
      while (t<=endTime){
        stateFunc.interpolate(t, state);
        geometry_msgs::Point comPosition;
        comPosition.x = state[3];
        comPosition.y = state[4];
        comPosition.z = state[5];
        comMarker.points.push_back(comPosition);

        computeFeetState(state, input_vector_t::Zero(), o_feetPositionRef, o_feetVelocityRef, o_feetForceRef);
        for (int i = 0; i < 4; i++) {
          geometry_msgs::Point footPosition;
          footPosition.x = o_feetPositionRef[i][0];
          footPosition.y = o_feetPositionRef[i][1];
          footPosition.z = o_feetPositionRef[i][2];
          footMarkerMsg.markers[i].points.push_back(footPosition);
        }
        if (t==endTime){break;}
        t=std::min(t+dt, endTime);
      }

      const auto q_world_base = quaternionBaseToOrigin<scalar_t>(state.template head<3>());
      geometry_msgs::Pose poseMsg;
      tf::quaternionEigenToMsg(q_world_base, poseMsg.orientation);
      poseMsg.position = comMarker.points.back();
      poseArray.poses.push_back(poseMsg);

      // comTracePublisher_.publish(comMarker);
      // feetTracePublisher_.publish(footMarkerMsg);
      comMPCTracePublisher_.publish(comMarker);
      feetMPCTracePublisher_.publish(footMarkerMsg);
      poseMPCPublisher_.publish(poseArray);
    }

}  // namespace switched_model
