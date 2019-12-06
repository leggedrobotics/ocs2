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

#include <robot_state_publisher/robot_state_publisher.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <urdf/model.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <kdl_parser/kdl_parser.hpp>

#include <ocs2_ballbot_example/BallbotParameters.h>
#include <ocs2_comm_interfaces/test/MRT_ROS_Dummy_Loop.h>
#include "ocs2_ballbot_example/definitions.h"

namespace ocs2 {
namespace ballbot {

class MRT_ROS_Dummy_Ballbot final : public MRT_ROS_Dummy_Loop<ballbot::STATE_DIM_, ballbot::INPUT_DIM_> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = MRT_ROS_Dummy_Loop<ballbot::STATE_DIM_, ballbot::INPUT_DIM_>;
  using ballbot_parameters_t = BallbotParameters<scalar_t>;

  /**
   * Constructor.
   *
   * @param [in] mrt: The underlying MRT class to be used.
   * @param [in] mrtDesiredFrequency: MRT loop frequency in Hz. This should always set to a positive number.
   * @param [in] mpcDesiredFrequency: MPC loop frequency in Hz. If set to a positive number, MPC loop
   * will be simulated to run by this frequency. Note that this might not be the MPC's realtime frequency.
   */
  MRT_ROS_Dummy_Ballbot(mrt_t& mrt, scalar_t mrtDesiredFrequency, scalar_t mpcDesiredFrequency)
      : BASE(mrt, mrtDesiredFrequency, mpcDesiredFrequency) {}

  /**
   * Destructor.
   */
  ~MRT_ROS_Dummy_Ballbot() override = default;

 protected:
  void launchVisualizerNode(int argc, char* argv[]) override {
    ros::init(argc, argv, "ballbot_visualization_node");

    ros::NodeHandle n;
    visualizationPublisher_ = n.advertise<visualization_msgs::MarkerArray>("ballbot_vis", 10);
    ROS_INFO_STREAM("Waiting for visualization subscriber ...");
    while (ros::ok() && visualizationPublisher_.getNumSubscribers() == 0) {
      ros::Rate(100).sleep();
    }
    ROS_INFO_STREAM("Visualization subscriber is connected.");

    // load a kdl-tree from the urdf robot description and initialize the robot state publisher
    std::string urdfName = "robot_description";
    urdf::Model model;
    if (!model.initParam(urdfName)) ROS_ERROR("URDF model load was NOT successful");
    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(model, tree)) {
      ROS_ERROR("Failed to extract kdl tree from xml robot description");
    }

    robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(tree));
    robotStatePublisherPtr_->publishFixedTransforms("", true);
    tfBroadcasterPtr_.reset(new tf::TransformBroadcaster);
  }

  void publishVisualizer(const system_observation_t& observation, const primal_solution_t& policy, const command_data_t& command) override {
    const auto& costDesiredTrajectories = command.mpcCostDesiredTrajectories_;

    // publish world transform
    ros::Time timeMsg = ros::Time::now();
    geometry_msgs::TransformStamped world_transform;
    world_transform.header.stamp = timeMsg;
    world_transform.header.frame_id = "odom";
    world_transform.child_frame_id = "world";
    world_transform.transform.translation.x = 0.0;
    world_transform.transform.translation.y = 0.0;
    world_transform.transform.translation.z = 0.0;
    world_transform.transform.rotation.x = 0.0;
    world_transform.transform.rotation.y = 0.0;
    world_transform.transform.rotation.z = 0.0;
    world_transform.transform.rotation.w = 1.0;
    tfBroadcasterPtr_->sendTransform(world_transform);

    // publish command transform
    const Eigen::Vector3d desiredPositionWorldToTarget = Eigen::Vector3d(costDesiredTrajectories.desiredStateTrajectory().back()(0),
                                                                         costDesiredTrajectories.desiredStateTrajectory().back()(1), 0.0);
    const Eigen::Quaterniond desiredQuaternionBaseToWorld =
        Eigen::AngleAxisd{costDesiredTrajectories.desiredStateTrajectory().back()(2), Eigen::Vector3d{0, 0, 1}} *
        Eigen::AngleAxisd{costDesiredTrajectories.desiredStateTrajectory().back()(3), Eigen::Vector3d{0, 1, 0}} *
        Eigen::AngleAxisd{costDesiredTrajectories.desiredStateTrajectory().back()(4), Eigen::Vector3d{1, 0, 0}};
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

    // publish joints transforms
    std::map<std::string, double> jointPositions;
    // initialize jointPositions to be used by the robot state publisher
    jointPositions.insert(std::pair<std::string, double>("jball_x", observation.state()(0)));
    jointPositions.insert(std::pair<std::string, double>("jball_y", observation.state()(1)));
    jointPositions.insert(std::pair<std::string, double>("jbase_z", observation.state()(2)));
    jointPositions.insert(std::pair<std::string, double>("jbase_y", observation.state()(3)));
    jointPositions.insert(std::pair<std::string, double>("jbase_x", observation.state()(4)));
    robotStatePublisherPtr_->publishTransforms(jointPositions, timeMsg, "");

    ros::spinOnce();
  }

  /************
   * Variables
   ************/
  ros::Publisher visualizationPublisher_;
  std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;
  std::unique_ptr<tf::TransformBroadcaster> tfBroadcasterPtr_;

  ballbot_parameters_t param_;
};

}  // namespace ballbot
}  // namespace ocs2
