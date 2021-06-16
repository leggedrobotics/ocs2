/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#pragma once

#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosInterface.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

namespace ocs2 {
namespace mobile_manipulator {

class TargetTrajectories_IMarker_Mobile_Manipulator : public TargetTrajectoriesRosInterface {
 public:
  TargetTrajectories_IMarker_Mobile_Manipulator(int argc, char* argv[], std::string robotName = "robot");
  ~TargetTrajectories_IMarker_Mobile_Manipulator() override = default;

 private:
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void observationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg);

  interactive_markers::InteractiveMarkerServer server;
  interactive_markers::MenuHandler menu_handler;

  ros::Subscriber observationSubscriber_;
  ocs2_msgs::mpc_observation::ConstPtr latestObservation_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
