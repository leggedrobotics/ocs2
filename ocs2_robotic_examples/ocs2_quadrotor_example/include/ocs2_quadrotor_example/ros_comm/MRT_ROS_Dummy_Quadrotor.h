/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include <tf/transform_broadcaster.h>

#include <ocs2_comm_interfaces/test/MRT_ROS_Dummy_Loop.h>
#include "ocs2_quadrotor_example/definitions.h"

namespace ocs2 {
namespace quadrotor {

class MRT_ROS_Dummy_Quadrotor final : public MRT_ROS_Dummy_Loop<quadrotor::STATE_DIM_, quadrotor::INPUT_DIM_> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = MRT_ROS_Dummy_Loop<quadrotor::STATE_DIM_, quadrotor::INPUT_DIM_>;

  /**
   * Constructor.
   *
   * @param [in] mrt: The underlying MRT class to be used.
   * @param [in] mrtDesiredFrequency: MRT loop frequency in Hz. This should always set to a positive number.
   * @param [in] mpcDesiredFrequency: MPC loop frequency in Hz. If set to a positive number, MPC loop
   * will be simulated to run by this frequency. Note that this might not be the MPC's realtime frequency.
   */
  MRT_ROS_Dummy_Quadrotor(mrt_t& mrt, scalar_t mrtDesiredFrequency, scalar_t mpcDesiredFrequency)
      : BASE(mrt, mrtDesiredFrequency, mpcDesiredFrequency) {}

  /**
   * Default destructor.
   */
  ~MRT_ROS_Dummy_Quadrotor() override = default;

 protected:
  void launchVisualizerNode(int argc, char* argv[]) override { tfBroadcasterPtr_.reset(new tf::TransformBroadcaster); }

  void publishVisualizer(const system_observation_t& observation, const primal_solution_t& policy, const command_data_t& command) override {
    const auto& costDesiredTrajectories = command.mpcCostDesiredTrajectories_;

    // publish command transform
    const Eigen::Vector3d desiredPositionWorldToTarget = Eigen::Vector3d(costDesiredTrajectories.desiredStateTrajectory().back()(0),
                                                                         costDesiredTrajectories.desiredStateTrajectory().back()(1),
                                                                         costDesiredTrajectories.desiredStateTrajectory().back()(2));
    const Eigen::Quaterniond desiredQuaternionBaseToWorld =
        Eigen::AngleAxisd{costDesiredTrajectories.desiredStateTrajectory().back()(3), Eigen::Vector3d{0, 0, 1}} *
        Eigen::AngleAxisd{costDesiredTrajectories.desiredStateTrajectory().back()(4), Eigen::Vector3d{0, 1, 0}} *
        Eigen::AngleAxisd{costDesiredTrajectories.desiredStateTrajectory().back()(5), Eigen::Vector3d{1, 0, 0}};
    ros::Time timeMsg = ros::Time::now();
    geometry_msgs::TransformStamped command_frame_transform;
    command_frame_transform.header.stamp = timeMsg;
    command_frame_transform.header.frame_id = "odom";
    command_frame_transform.child_frame_id = "command";
    command_frame_transform.transform.translation.x = desiredPositionWorldToTarget.x();
    command_frame_transform.transform.translation.y = desiredPositionWorldToTarget.y();
    command_frame_transform.transform.translation.z = desiredPositionWorldToTarget.z();
    command_frame_transform.transform.rotation.w = desiredQuaternionBaseToWorld.w();
    command_frame_transform.transform.rotation.x = desiredQuaternionBaseToWorld.x();
    command_frame_transform.transform.rotation.y = desiredQuaternionBaseToWorld.y();
    command_frame_transform.transform.rotation.z = desiredQuaternionBaseToWorld.z();
    tfBroadcasterPtr_->sendTransform(command_frame_transform);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(observation.state()(0), observation.state()(1), observation.state()(2)));
    tf::Quaternion q = tf::createQuaternionFromRPY(observation.state()(3), observation.state()(4), observation.state()(5));
    transform.setRotation(q);
    tfBroadcasterPtr_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base"));
  }

  /************
   * Variables
   ************/
  std::unique_ptr<tf::TransformBroadcaster> tfBroadcasterPtr_;
};

}  // namespace quadrotor
}  // namespace ocs2
