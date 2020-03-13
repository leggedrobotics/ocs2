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

#include <Eigen/Dense>

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/control/ControllerBase.h>
#include <ocs2_core/cost/CostDesiredTrajectories.h>
#include <ocs2_core/logic/ModeSchedule.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>
#include <ocs2_oc/rollout/RolloutBase.h>

#include "ocs2_comm_interfaces/CommandData.h"
#include "ocs2_comm_interfaces/SystemObservation.h"

namespace ocs2 {

/**
 * This class implements core MRT (Model Reference Tracking) functionality.
 * The responsibility of filling the buffer variables is left to the deriving classes.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class MRT_BASE {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using dim_t = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_array_t = typename dim_t::scalar_array_t;
  using scalar_t = typename dim_t::scalar_t;
  using size_array_t = typename dim_t::size_array_t;
  using state_vector_t = typename dim_t::state_vector_t;
  using state_vector_array_t = typename dim_t::state_vector_array_t;
  using input_vector_t = typename dim_t::input_vector_t;
  using input_vector_array_t = typename dim_t::input_vector_array_t;

  using controller_t = ControllerBase<STATE_DIM, INPUT_DIM>;
  using rollout_base_t = RolloutBase<STATE_DIM, INPUT_DIM>;

  using primal_solution_t = PrimalSolution<STATE_DIM, INPUT_DIM>;
  using command_data_t = CommandData<STATE_DIM, INPUT_DIM>;

  /**
   * Constructor
   */
  MRT_BASE();

  /**
   * @brief Default destructor
   */
  ~MRT_BASE() = default;

  /**
   * Resets the class to its instantiated state.
   */
  void reset();

  /**
   * Request the MPC node to reset. This method is a blocking method.
   *
   * @param [in] initCostDesiredTrajectories: The initial desired cost trajectories.
   */
  virtual void resetMpcNode(const CostDesiredTrajectories& initCostDesiredTrajectories) = 0;

  /**
   * Whether the initial MPC policy has been already received.
   */
  bool initialPolicyReceived() const { return policyReceivedEver_; }

  /**
   * @brief setCurrentObservation notifies MPC of a new state
   * @param observation: the current measurement to send to the MPC
   */
  virtual void setCurrentObservation(const SystemObservation<STATE_DIM, INPUT_DIM>& observation) = 0;

  /**
   * Gets a reference to the command data corresponding to the current policy.
   * @warning access to the returned reference is not threadsafe. Read access and calls to updatePolicy() must be synced by the user.
   *
   * @return a constant reference to command data.
   */
  const command_data_t& getCommand() const { return *currentCommand_; };

  /**
   * Gets a reference to current optimized policy.
   * @return constant reference to the policy data.
   */
  const primal_solution_t& getPolicy() const { return *currentPrimalSolution_; };

  /**
   * @brief Initializes rollout class to roll out a feedback policy
   * @param rolloutPtr: The rollout object to be used
   */
  void initRollout(const rollout_base_t* rolloutPtr);

  /**
   * @brief Evaluates the controller
   *
   * @param [in] currentTime: the query time.
   * @param [in] currentState: the query state.
   * @param [out] mpcState: the current nominal state of MPC.
   * @param [out] mpcInput: the optimized control input.
   * @param [out] mode: the active mode.
   */
  void evaluatePolicy(scalar_t currentTime, const state_vector_t& currentState, state_vector_t& mpcState, input_vector_t& mpcInput,
                      size_t& mode);

  /**
   * @brief Rolls out the control policy from the current time and state to get the next state and input using the MPC policy.
   *
   * @param [in] currentTime: start time of the rollout.
   * @param [in] currentState: state to start rollout from.
   * @param [in] timeStep: duration of the forward rollout.
   * @param [out] mpcState: the new forwarded state of MPC.
   * @param [out] mpcInput: the new control input of MPC.
   * @param [out] mode: the active mode.
   */
  void rolloutPolicy(scalar_t currentTime, const state_vector_t& currentState, const scalar_t& timeStep, state_vector_t& mpcState,
                     input_vector_t& mpcInput, size_t& mode);

  /**
   * Checks the data buffer for an update of the MPC policy. If a new policy
   * is available on the buffer this method will load it to the in-use policy.
   * This method also calls the modifyPolicy() method.
   *
   * @return True if the policy is updated.
   */
  bool updatePolicy();

  /**
   * @brief rolloutSet: Whether or not the internal rollout object has been set
   * @return True if a rollout object is available.
   */
  bool isRolloutSet() const { return rolloutPtr_.get(); }

 protected:
  /**
   * The updatePolicy() method will call this method which allows the user to
   * customize the in-use policy. Note that this method is already
   * protected with a mutex which blocks the policy callback. Moreover, this method
   * may be called in the main thread of the program. Thus, for efficiency and
   * practical considerations you should avoid computationally expensive operations.
   * For such operations you may want to use the modifyBufferPolicy()
   * methods which runs on a separate thread which directly modifies the received
   * policy messages on the data buffer.
   *
   */
  virtual void modifyPolicy(const command_data_t& command, primal_solution_t& primalSolution) {}

  /**
   * This method can be used to modify the policy on the buffer without inputting the main thread.
   *
   * @param [in] commandBuffer: buffered command data.
   * @param primalSolutionBuffer: The primal problem's solution on the buffer.
   */
  virtual void modifyBufferPolicy(const command_data_t& commandBuffer, primal_solution_t& primalSolutionBuffer) {}

  /**
   * Constructs a partitioningTimes vector with 2 elements: minimum of the already
   * received times and the maximum value of the numeric type scalar_t. This prevents
   * the frequent update of the logicRules.
   *
   * @param [in] time: The current time.
   * @param [out] partitioningTimes: Partitioning time.
   */
  void partitioningTimesUpdate(scalar_t time, scalar_array_t& partitioningTimes) const;

 protected:
  // flags on state of the class
  std::atomic_bool policyReceivedEver_;
  bool newPolicyInBuffer_;  //! Whether a new policy is waiting to be swapped in

  // variables related to the MPC output
  std::atomic_bool policyUpdated_;  //! Whether the policy was updated by MPC (i.e., MPC succeeded)
  bool policyUpdatedBuffer_;        //! Whether the policy in buffer was upated by MPC (i.e., MPC succeeded)
  std::unique_ptr<primal_solution_t> currentPrimalSolution_;
  std::unique_ptr<primal_solution_t> primalSolutionBuffer_;
  std::unique_ptr<command_data_t> currentCommand_;
  std::unique_ptr<command_data_t> commandBuffer_;

  // thread safety
  mutable std::mutex policyBufferMutex_;  // for policy variables WITH suffix (*Buffer_)

  // variables needed for policy evaluation
  std::unique_ptr<rollout_base_t> rolloutPtr_;

  // variables
  scalar_array_t partitioningTimes_;
  scalar_array_t partitioningTimesBuffer_;
  SystemObservation<STATE_DIM, INPUT_DIM> initPlanObservation_;  //! The initial observation of the first plan ever received
};

}  // namespace ocs2

#include "implementation/MRT_BASE.h"
