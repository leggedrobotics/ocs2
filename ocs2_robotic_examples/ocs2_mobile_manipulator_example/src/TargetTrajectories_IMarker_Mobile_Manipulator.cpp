/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <ocs2_mobile_manipulator_example/TargetTrajectories_IMarker_Mobile_Manipulator.h>
#include <ocs2_mobile_manipulator_example/definitions.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TargetTrajectories_IMarker_Mobile_Manipulator::TargetTrajectories_IMarker_Mobile_Manipulator(int argc, char* argv[], std::string robotName)
    : TargetTrajectoriesRosInterface(argc, argv, robotName), server("simple_marker") {
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
  box_control.always_visible = 1;
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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TargetTrajectories_IMarker_Mobile_Manipulator::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
  SystemObservation observation;
  ros_msg_conversions::readObservationMsg(*latestObservation_, observation);

  // Desired time trajectory
  TargetTrajectories targetTrajectories(1);
  scalar_array_t& tDesiredTrajectory = targetTrajectories.timeTrajectory;
  tDesiredTrajectory[0] = observation.time;

  // Desired state trajectory
  vector_array_t& xDesiredTrajectory = targetTrajectories.stateTrajectory;
  xDesiredTrajectory[0].resize(7);  // 3 + 4 for desired position vector and orientation quaternion
  xDesiredTrajectory[0].tail<4>() = Eigen::Quaterniond(feedback->pose.orientation.w, feedback->pose.orientation.x,
                                                       feedback->pose.orientation.y, feedback->pose.orientation.z)
                                        .coeffs();
  xDesiredTrajectory[0].head<3>() << feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z;

  // Desired input trajectory
  vector_array_t& uDesiredTrajectory = targetTrajectories.inputTrajectory;
  uDesiredTrajectory[0].setZero(INPUT_DIM);

  this->publishTargetTrajectories(targetTrajectories);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TargetTrajectories_IMarker_Mobile_Manipulator::observationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg) {
  latestObservation_ = msg;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
