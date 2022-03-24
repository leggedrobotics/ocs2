/******************************************************************************
Copyright (c) 2022, Farbod Farshidian. All rights reserved.

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

#include <ros/ros.h>

#include <ocs2_ros_interfaces/mrt/DummyObserver.h>

namespace ocs2 {
namespace mpcnet {

/**
 *  Dummy observer that publishes the current system observation that is required for some target command nodes.
 */
class MpcnetDummyObserverRos : public DummyObserver {
 public:
  /**
   * Constructor.
   * @param [in] nodeHandle : The ROS node handle.
   * @param [in] topicPrefix : The prefix defines the names for the observation's publishing topic "topicPrefix_mpc_observation".
   */
  explicit MpcnetDummyObserverRos(ros::NodeHandle& nodeHandle, std::string topicPrefix = "anonymousRobot");

  /**
   * Default destructor.
   */
  ~MpcnetDummyObserverRos() override = default;

  /**
   * Update and publish.
   * @param [in] observation: The current system observation.
   * @param [in] primalSolution: The current primal solution.
   * @param [in] command: The given command data.
   */
  void update(const SystemObservation& observation, const PrimalSolution& primalSolution, const CommandData& command) override;

 private:
  ros::Publisher observationPublisher_;
};

}  // namespace mpcnet
}  // namespace ocs2
