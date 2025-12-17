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

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h"

#include "ocs2_ros_interfaces/common/RosMsgConversions.h"
#include <rclcpp/rclcpp.hpp>

// MPC messages
#include <ocs2_msgs/msg/mode_schedule.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
RosReferenceManager::RosReferenceManager(
    std::string topicPrefix,
    std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr)
    : ReferenceManagerDecorator(std::move(referenceManagerPtr)),
      topicPrefix_(std::move(topicPrefix)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RosReferenceManager::subscribe(const rclcpp::Node::SharedPtr& node) {
  node_ = node;
  // ModeSchedule
  auto modeScheduleCallback = [this](const ocs2_msgs::msg::ModeSchedule& msg) {
    auto modeSchedule = ros_msg_conversions::readModeScheduleMsg(msg);
    referenceManagerPtr_->setModeSchedule(std::move(modeSchedule));
  };
  modeScheduleSubscriber_ =
      node_->create_subscription<ocs2_msgs::msg::ModeSchedule>(
          topicPrefix_ + "_mode_schedule", 1, modeScheduleCallback);

  // TargetTrajectories
  auto targetTrajectoriesCallback =
      [this](const ocs2_msgs::msg::MpcTargetTrajectories& msg) {
        auto targetTrajectories =
            ros_msg_conversions::readTargetTrajectoriesMsg(msg);
        referenceManagerPtr_->setTargetTrajectories(
            std::move(targetTrajectories));
      };
  targetTrajectoriesSubscriber_ =
      node_->create_subscription<ocs2_msgs::msg::MpcTargetTrajectories>(
          topicPrefix_ + "_mpc_target", 1, targetTrajectoriesCallback);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RosReferenceManager::subscribe_act(const rclcpp::Node::SharedPtr& node, PinocchioInterface pinocchioInterface, int eeFrameId, const std::string& armSide) {
  node_ = node;
  
  // ModeSchedule
  auto modeScheduleCallback = [this](const ocs2_msgs::msg::ModeSchedule& msg) {
    auto modeSchedule = ros_msg_conversions::readModeScheduleMsg(msg);
    referenceManagerPtr_->setModeSchedule(std::move(modeSchedule));
  };
  modeScheduleSubscriber_ =
      node_->create_subscription<ocs2_msgs::msg::ModeSchedule>(
          topicPrefix_ + "_mode_schedule", 1, modeScheduleCallback);

  // get robot model for forward kinematics
  auto model = pinocchioInterface.getModel(); 

  // TargetTrajectories
  auto actTargetTrajectoriesCallback = [this, model, eeFrameId, armSide](const std_msgs::msg::Float32MultiArray& msg) {
    // msg = [ left_arm_qpos (6),          # absolute joint position
    //         left_gripper_position (1),  # normalized gripper position (0: close, 1: open)
    //         right_arm_qpos (6),         # absolute joint position
    //         right_gripper_qpos (1),     # normalized gripper position (0: close, 1: open)
    //         ros_time_stamp ]            # rospy.Time.now().to_sec()
    auto actTargetQpos = ros_msg_conversions::readActTargetQposMsg(msg);
    // Split into left arm and right arm
    Eigen::VectorXd armTargetQpos;
    if (armSide == "LEFT") {
      armTargetQpos = actTargetQpos.head(6);  // left_arm_qpos
    } else if (armSide == "RIGHT") {
      armTargetQpos = actTargetQpos.segment(7, 6);  // right_arm_qpos
    } else {
      std::cerr << "[Error] Invalid armSide parameter " << armSide << ". Use LEFT or RIGHT.\n";
      return;
    }
    
    pinocchio::Data data(model);
    pinocchio::forwardKinematics(model, data, armTargetQpos);
    pinocchio::updateFramePlacements(model, data);

    // calculate end frame quaternion
    Eigen::Quaterniond quaternion(data.oMf[eeFrameId].rotation());
    const vector_t target = (vector_t(7) << data.oMf[eeFrameId].translation(), quaternion.coeffs()).finished();

    // set time stamp
    const scalar_array_t desiredTimeTrajectory{actTargetQpos[14]}; 
    // set end frame position and quaternion
    const vector_array_t desiredStateTrajectory{target};
    // set input trajectory all zero
    const vector_array_t desiredInputTrajectory{vector_t::Zero(model.nv)};
    TargetTrajectories targetTrajectories(desiredTimeTrajectory, desiredStateTrajectory, desiredInputTrajectory);

    referenceManagerPtr_->setTargetTrajectories(std::move(targetTrajectories));
  };

  actTargetTrajectoriesSubscriber_ =
      node_->create_subscription<std_msgs::msg::Float32MultiArray>(
          "/act_aloha_target_qpos", 1, actTargetTrajectoriesCallback);
}

}  // namespace ocs2
