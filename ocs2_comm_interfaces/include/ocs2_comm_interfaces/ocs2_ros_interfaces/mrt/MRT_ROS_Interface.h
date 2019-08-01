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

#include <ocs2_comm_interfaces/ocs2_interfaces/MRT_BASE.h>

#include <chrono>
#include <condition_variable>
#include <csignal>
#include <ctime>
#include <iostream>
#include <string>
#include <thread>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>

#include <ocs2_core/logic/machine/HybridLogicRulesMachine.h>
#include <ocs2_core/logic/rules/NullLogicRules.h>

// MPC messages
#include <ocs2_comm_interfaces/dummy.h>
#include <ocs2_comm_interfaces/mpc_flattened_controller.h>
#include <ocs2_comm_interfaces/ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_comm_interfaces/reset.h>

#define PUBLISH_THREAD

namespace ocs2 {

/**
 * This class implements MRT (Model Reference Tracking) communication interface using ROS.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class MRT_ROS_Interface : public MRT_BASE<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = MRT_BASE<STATE_DIM, INPUT_DIM>;

  using Ptr = std::shared_ptr<MRT_ROS_Interface<STATE_DIM, INPUT_DIM>>;

  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using size_array_t = typename DIMENSIONS::size_array_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
  using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t;
  using input_state_matrix_array_t = typename DIMENSIONS::input_state_matrix_array_t;

  using ros_msg_conversions_t = RosMsgConversions<STATE_DIM, INPUT_DIM>;
  using cost_desired_trajectories_t = CostDesiredTrajectories<scalar_t>;
  using rollout_base_ptr_t = typename std::unique_ptr<RolloutBase<STATE_DIM, INPUT_DIM>>;
  using time_triggered_rollout_t = TimeTriggeredRollout<STATE_DIM, INPUT_DIM>;
  using controlled_system_base_t = ControlledSystemBase<STATE_DIM, INPUT_DIM>;
  using system_observation_t = SystemObservation<STATE_DIM, INPUT_DIM>;
  using state_linear_interpolation_t = LinearInterpolation<state_vector_t, Eigen::aligned_allocator<state_vector_t>>;
  using controller_t = ControllerBase<STATE_DIM, INPUT_DIM>;

  /**
   * Constructor
   *
   * @param [in] robotName: The robot's name.
   * @param [in] logicRules: A logic rule class of derived from the hybrid logicRules base.
   */
  explicit MRT_ROS_Interface(std::string robotName = "robot", std::shared_ptr<HybridLogicRules> logicRules = nullptr);

  /**
   * Destructor
   */
  virtual ~MRT_ROS_Interface();

  void reset() override;

  void resetMpcNode(const cost_desired_trajectories_t& initCostDesiredTrajectories) override;

  /**
   * Shut down the ROS nodes.
   */
  void shutdownNodes();

  /**
   * Shut down publisher
   */
  void shutdownPublisher();

  /**
   * spin the MRT callback queue
   */
  void spinMRT();

  /**
   * Launches the ROS nodes to communicate with the MPC node.
   *
   * @param [in] argc: Command line number of arguments.
   * @param [in] argv: Command line vector of arguments.
   */
  void launchNodes(int argc, char* argv[]);

  /**
   *  Gets the node handle pointer to the MRT node,
   *  Use this to add subscribers to the custom MRT callback queue
   */
  ::ros::NodeHandlePtr& nodeHandle();

  /**
   * Publishes the current observation on a separate thread
   * without blocking the main thread.
   *
   * @param [in] currentObservation: The current observation.
   */
  void publishObservation(const system_observation_t& currentObservation);

 protected:
  /**
   * Signal handler
   *
   * @param sig: signal
   */
  static void sigintHandler(int sig);

  /**
   * Dummy publisher for network debugging.
   */
  void publishDummy();

  /**
   * Callback method to receive the MPC policy as well as the mode sequence.
   * It only updates the policy variables with suffix (*Buffer_) variables.
   *
   * @param [in] msg: A constant pointer to the message
   */
  void mpcPolicyCallback(const ocs2_comm_interfaces::mpc_flattened_controller::ConstPtr& msg);

  /**
   * A thread function which sends the current state and checks for a new MPC update.
   */
  void publisherWorkerThread();

 protected:
  std::string robotName_;

  ::ros::NodeHandlePtr mrtRosNodeHandlePtr_;

  // Publishers and subscribers
  ::ros::Publisher dummyPublisher_;
  ::ros::Publisher mpcObservationPublisher_;
  ::ros::Subscriber mpcPolicySubscriber_;
  ::ros::ServiceClient mpcResetServiceClient_;

  // ROS messages
  ocs2_comm_interfaces::mpc_observation mpcObservationMsg_;
  ocs2_comm_interfaces::mpc_observation mpcObservationMsgBuffer_;

  ::ros::CallbackQueue mrtCallbackQueue_;

  // Multi-threading for publishers
  bool terminateThread_;
  bool readyToPublish_;
  std::thread publisherWorker_;
  std::mutex publisherMutex_;
  std::condition_variable msgReady_;
};

}  // namespace ocs2

#include "implementation/MRT_ROS_Interface.h"
