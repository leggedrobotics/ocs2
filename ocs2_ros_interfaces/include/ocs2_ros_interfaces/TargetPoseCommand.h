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

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <csignal>
#include <iostream>
#include <string>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace switched_model {

class TargetPoseCommand {
 public:
  using scalar_t = double;
  using scalar_array_t = std::vector<scalar_t>;

  explicit TargetPoseCommand(std::string robotName, scalar_array_t goalPoseLimit = scalar_array_t{2.0, 1.0, 0.3, 45.0, 45.0, 360.0})

      : robotName_(std::move(robotName)), goalPoseLimit_(std::move(goalPoseLimit)) {
    if (goalPoseLimit_.size() != maxCommandSize_) {
      throw std::runtime_error("The goal pose limit should be of size 6.");
    }
  }

  ~TargetPoseCommand() = default;

  void launchNodes(int argc, char* argv[]) {
    ros::init(argc, argv, "target_pose");
    ros::NodeHandle nodeHandler;
    gaolPublisher_ = nodeHandler.advertise<geometry_msgs::Pose>(robotName_ + "_target", 1);
    ros::spinOnce();

    // display
    ROS_INFO_STREAM(robotName_ + " 	command node is ready.");
  }

  void run() {
    auto deg2rad = [](const scalar_t& deg) { return (deg * M_PI / 180); };
    auto rad2deg = [](const scalar_t& rad) { return (rad * 180 / M_PI); };

    while (ros::ok()) {
      commadInput_.clear();

      std::cout << "Enter XYZ displacement for the robot, separated by spaces: ";
      std::string line;
      std::getline(std::cin, line);
      std::istringstream stream(line);
      scalar_t in;
      while (stream >> in) {
        commadInput_.push_back(in);
      }

      if (commadInput_.size() > maxCommandSize_) {
        commadInput_.erase(commadInput_.begin() + maxCommandSize_, commadInput_.end());
      }

      for (size_t i = 0; i < maxCommandSize_; i++) {
        if (i + 1 > commadInput_.size()) {
          commadInput_.push_back(0.0);
          continue;
        }
        if (std::abs(commadInput_[i]) > goalPoseLimit_[i]) {
          commadInput_[i] = (commadInput_[i] > 0) ? goalPoseLimit_[i] : -goalPoseLimit_[i];
        }
      }  // end of i loop

      // creat the message
      geometry_msgs::Pose basePoseMsg;
      basePoseMsg.position.x = commadInput_[0];
      basePoseMsg.position.y = commadInput_[1];
      basePoseMsg.position.z = commadInput_[2];

      Eigen::Quaternion<scalar_t> qxyz = Eigen::AngleAxis<scalar_t>(deg2rad(commadInput_[3]), Eigen::Vector3d::UnitX()) *  // roll
                                         Eigen::AngleAxis<scalar_t>(deg2rad(commadInput_[4]), Eigen::Vector3d::UnitY()) *  // pitch
                                         Eigen::AngleAxis<scalar_t>(deg2rad(commadInput_[5]), Eigen::Vector3d::UnitZ());   // yaw

      basePoseMsg.orientation.x = qxyz.x();
      basePoseMsg.orientation.y = qxyz.y();
      basePoseMsg.orientation.z = qxyz.z();
      basePoseMsg.orientation.w = qxyz.w();

      gaolPublisher_.publish(basePoseMsg);

      std::cout << "The following position displacement is published: [";
      for (size_t i = 0; i < maxCommandSize_; i++) {
        std::cout << commadInput_[i] << ", ";
      }
      std::cout << "\b\b]" << std::endl << std::endl;

    }  // enf of while loop
  }

 private:
  const size_t maxCommandSize_ = 6;

  std::string robotName_;

  scalar_array_t goalPoseLimit_;

  ros::Publisher gaolPublisher_;

  scalar_array_t commadInput_;
};

}  // end of namespace switched_model
