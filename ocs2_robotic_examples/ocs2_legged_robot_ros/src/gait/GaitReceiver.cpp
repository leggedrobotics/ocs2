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

#include "ocs2_legged_robot_ros/gait/GaitReceiver.h"

#include "ocs2_legged_robot_ros/gait/ModeSequenceTemplateRos.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GaitReceiver::GaitReceiver(const rclcpp::Node::SharedPtr& node,
                           std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                           const std::string& robotName)
    : gaitSchedulePtr_(std::move(gaitSchedulePtr)),
      receivedGait_({0.0, 1.0}, {ModeNumber::STANCE}),
      gaitUpdated_(false) {
  mpcModeSequenceSubscriber_ =
      node->create_subscription<ocs2_msgs::msg::ModeSchedule>(
          robotName + "_mpc_mode_schedule", 1,
          std::bind(&GaitReceiver::mpcModeSequenceCallback, this,
                    std::placeholders::_1));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitReceiver::preSolverRun(
    scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
    const ReferenceManagerInterface& referenceManager) {
  if (gaitUpdated_) {
    std::lock_guard<std::mutex> lock(receivedGaitMutex_);
    std::cerr << "[GaitReceiver]: Setting new gait after time " << finalTime
              << "\n";
    std::cerr << receivedGait_;
    const auto timeHorizon = finalTime - initTime;
    gaitSchedulePtr_->insertModeSequenceTemplate(receivedGait_, finalTime,
                                                 timeHorizon);
    gaitUpdated_ = false;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitReceiver::mpcModeSequenceCallback(
    const ocs2_msgs::msg::ModeSchedule::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lock(receivedGaitMutex_);
  receivedGait_ = readModeSequenceTemplateMsg(*msg);
  gaitUpdated_ = true;
}

}  // namespace legged_robot
}  // namespace ocs2
