/*
 * TargetTrajectories_IMarker_Mobile_Manipulator.h
 *
 *  Created on: 31 Aug 2020
 *      Author: perry
 */

#pragma once

#include <ocs2_ros_interfaces/command/TargetTrajectories_ROS_Interface.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
namespace mobile_manipulator {

class TargetTrajectories_IMarker_Mobile_Manipulator : public ocs2::TargetTrajectories_ROS_Interface {
 public:
  TargetTrajectories_IMarker_Mobile_Manipulator(int argc, char* argv[], std::string robotName = "robot");
  virtual ~TargetTrajectories_IMarker_Mobile_Manipulator() = default;

 private:
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void observationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg);

  interactive_markers::InteractiveMarkerServer server;
  interactive_markers::MenuHandler menu_handler;

  ros::Subscriber observationSubscriber_;
  ocs2_msgs::mpc_observation::ConstPtr latestObservation_;
};

} /* namespace mobile_manipulator */
