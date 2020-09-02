/*
 * TargetTrajectories_IMarker_Mobile_Manipulator.cpp
 *
 *  Created on: 31 Aug 2020
 *      Author: perry
 */

#include <ocs2_mobile_manipulator_example/TargetTrajectories_IMarker_Mobile_Manipulator.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>

namespace mobile_manipulator {

TargetTrajectories_IMarker_Mobile_Manipulator::TargetTrajectories_IMarker_Mobile_Manipulator(int argc, char* argv[], std::string robotName)
    : ocs2::TargetTrajectories_ROS_Interface(argc, argv, robotName), server("simple_marker") {
  observationSubscriber_ = this->nodeHandle_->subscribe("/" + robotName + "_mpc_observation", 1,
                                                        &TargetTrajectories_IMarker_Mobile_Manipulator::observationCallback, this);
  // create an interactive marker for our server

  menu_handler.insert("First Entry", boost::bind(&TargetTrajectories_IMarker_Mobile_Manipulator::processFeedback, this, _1));

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "world";
  int_marker.header.stamp = ros::Time::now();
  int_marker.name = "Goal";
  int_marker.scale = 0.2;
  int_marker.description = "Right click to send command";
  int_marker.pose.position.z = 1.0;

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 0.1;
  box_marker.scale.y = 0.1;
  box_marker.scale.z = 0.1;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 0.5;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back(box_marker);
  box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

  // add the control to the interactive marker
  int_marker.controls.push_back(box_control);

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server.insert(int_marker);  //, boost::bind(&TargetTrajectories_IMarker_Mobile_Manipulator::processFeedback, this, _1));
  menu_handler.apply(server, int_marker.name);

  // 'commit' changes and send to all clients
  server.applyChanges();
}

void TargetTrajectories_IMarker_Mobile_Manipulator::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
  ocs2::SystemObservation observation;
  ocs2::ros_msg_conversions::readObservationMsg(*latestObservation_, observation);

  // Target reaching duration
  const ocs2::scalar_t averageSpeed = 2.0;
  ocs2::scalar_t targetReachingDuration = 10;  // relativeState.template head<3>().norm() / averageSpeed;

  // Desired time trajectory
  ocs2::CostDesiredTrajectories costDesiredTrajectories(2);
  ocs2::scalar_array_t& tDesiredTrajectory = costDesiredTrajectories.desiredTimeTrajectory();
  tDesiredTrajectory.resize(1);
  tDesiredTrajectory[0] = observation.time;
  //  tDesiredTrajectory[1] = observation.time + targetReachingDuration;

  // Desired state trajectory
  ocs2::vector_array_t& xDesiredTrajectory = costDesiredTrajectories.desiredStateTrajectory();
  xDesiredTrajectory.resize(1);
  //  xDesiredTrajectory[0] = observation.state;
  xDesiredTrajectory[0].resize(7);
  xDesiredTrajectory[0].template tail<4>() = Eigen::Quaterniond(feedback->pose.orientation.w, feedback->pose.orientation.x,
                                                                feedback->pose.orientation.y, feedback->pose.orientation.z)
                                                 .coeffs();
  xDesiredTrajectory[0].template head<3>() << feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z;

  // Desired input trajectory
  ocs2::vector_array_t& uDesiredTrajectory = costDesiredTrajectories.desiredInputTrajectory();
  uDesiredTrajectory.resize(1);
  uDesiredTrajectory[0].setZero(8);
  //  uDesiredTrajectory[1].setZero(8);

  this->publishTargetTrajectories(costDesiredTrajectories);
}

void TargetTrajectories_IMarker_Mobile_Manipulator::observationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg) {
  latestObservation_ = msg;
}

} /* namespace mobile_manipulator */
