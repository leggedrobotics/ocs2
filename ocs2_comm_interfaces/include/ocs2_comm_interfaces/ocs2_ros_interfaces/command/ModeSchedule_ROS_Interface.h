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

#include <string>
#include <vector>

#include <ros/ros.h>

// MPC messages
#include <ocs2_msgs/mode_schedule.h>

#include "ocs2_comm_interfaces/ocs2_ros_interfaces/common/RosMsgConversions.h"

namespace ocs2 {

/**
 * This class implements ModeSchedule communication interface using ROS.
 *
 * @tparam SCALAR_T: scalar type.
 */
template <typename SCALAR_T>
class ModeSchedule_ROS_Interface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using scalar_t = SCALAR_T;

  /**
   * Constructor.
   *
   * @param [in] robotName: The robot's name.
   */
  explicit ModeSchedule_ROS_Interface(std::string robotName = "robot");

  /**
   * Destructor.
   */
  virtual ~ModeSchedule_ROS_Interface();

  /**
   * This is the main routine which launches the publisher node for MPC's
   * desired trajectories.
   *
   * @param [in] argc: Command line number of arguments.
   * @param [in] argv: Command line vector of arguments.
   */
  void launchNodes(int argc, char* argv[]);

  /**
   * Publishes the mode schedule.
   *
   * @param [in] modeSchedule: The mode schedule.
   */
  void publishModeSchedule(const ModeSchedule& modeSchedule);

 protected:
  std::string robotName_;

  // Publisher
  ::ros::Publisher mpcModeSchedulePublisher_;
};

}  // namespace ocs2

#include "implementation/ModeSchedule_ROS_Interface.h"
