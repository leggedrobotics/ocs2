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

#include <condition_variable>
#include <csignal>
#include <ctime>
#include <iostream>
#include <string>
#include <thread>

#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_mpc/MPC_BASE.h>

#include "ocs2_comm_interfaces/ocs2_interfaces/MRT_BASE.h"

namespace ocs2 {

/**
 * A lean ROS independent interface to OCS2. In incorporates the functionality of the MPC and the MRT (trajectory tracking) modules.
 * Please refer to ocs2_double_integrator_example for a minimal example and tests
 * @tparam STATE_DIM
 * @tparam INPUT_DIM
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class MPC_MRT_Interface final : public MRT_BASE<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = MRT_BASE<STATE_DIM, INPUT_DIM>;
  using typename Base::command_data_t;
  using typename Base::primal_solution_t;

  using Ptr = std::shared_ptr<MPC_MRT_Interface<STATE_DIM, INPUT_DIM>>;

  using mpc_t = MPC_BASE<STATE_DIM, INPUT_DIM>;

  using scalar_t = typename mpc_t::scalar_t;
  using scalar_array_t = typename mpc_t::scalar_array_t;
  using scalar_array2_t = typename mpc_t::scalar_array2_t;
  using size_array_t = typename mpc_t::size_array_t;
  using state_vector_t = typename mpc_t::state_vector_t;
  using state_vector_array_t = typename mpc_t::state_vector_array_t;
  using state_vector_array2_t = typename mpc_t::state_vector_array2_t;
  using input_vector_t = typename mpc_t::input_vector_t;
  using input_vector_array_t = typename mpc_t::input_vector_array_t;
  using input_vector_array2_t = typename mpc_t::input_vector_array2_t;
  using controller_t = typename mpc_t::controller_t;
  using controller_ptr_array_t = typename mpc_t::controller_ptr_array_t;
  using input_state_matrix_t = typename mpc_t::input_state_matrix_t;
  using input_state_matrix_array_t = typename mpc_t::input_state_matrix_array_t;
  using dynamic_vector_t = typename mpc_t::dynamic_vector_t;

  using system_observation_t = SystemObservation<STATE_DIM, INPUT_DIM>;

  using state_linear_interpolation_t = EigenLinearInterpolation<state_vector_t>;
  using input_linear_interpolation_t = EigenLinearInterpolation<input_vector_t>;

  /**
   * Constructor
   * @param [in] mpc: The underlying MPC class to be used.
   */
  explicit MPC_MRT_Interface(mpc_t& mpc);

  /**
   * Destructor.
   */
  virtual ~MPC_MRT_Interface() = default;

  void resetMpcNode(const CostDesiredTrajectories& initCostDesiredTrajectories) override;

  void setCurrentObservation(const system_observation_t& currentObservation) override;

  /**
   * Set new target trajectories to be tracked.
   * It is safe to set a new value while the MPC optimization is running
   * @param targetTrajectories
   */
  void setTargetTrajectories(const CostDesiredTrajectories& targetTrajectories);

  /**
   * Advance the mpc module for one iteration.
   * The evaluation methods can be called while this method is running.
   * They will evaluate the control law that was up-to-date at the last updatePolicy() call
   */
  void advanceMpc();

  /**
   * @brief Access the solver's internal value function
   * @param time query time
   * @param state query state
   * @return value of the given state at the given time
   */
  scalar_t getValueFunction(scalar_t time, const state_vector_t& state);

  /**
   * @brief Calculates the state derivative of the value function
   * @param [in] time the query time
   * @param [out] Vx partial derivative of the value function at requested time at nominal state
   */
  void getValueFunctionStateDerivative(scalar_t time, const state_vector_t& state, state_vector_t& Vx);

  /**
   * @brief getLinearFeedbackGain retrieves K matrix from solver
   * @param [in] time
   * @param [out] K
   */
  void getLinearFeedbackGain(scalar_t time, input_state_matrix_t& K);

  /**
   * @brief Computes the Lagrange multiplier related to the state-input constraints
   * @param [in] time: query time
   * @param [in] state: query state
   * @param [out] nu: the Lagrange multiplier
   */
  void getStateInputConstraintLagrangian(scalar_t time, const state_vector_t& state, dynamic_vector_t& nu) const;

 protected:
  /**
   * @brief fillMpcOutputBuffers updates the *Buffer variables from the MPC object.
   * This method is automatically called by advanceMpc()
   * @param [in] mpcInitObservation: The observation used to run the MPC.
   */
  void fillMpcOutputBuffers(system_observation_t mpcInitObservation);

 protected:
  mpc_t& mpc_;

  benchmark::RepeatedTimer mpcTimer_;

  // MPC inputs
  system_observation_t currentObservation_;
  std::mutex observationMutex_;
  std::mutex costDesiredTrajectoriesBufferMutex_;
  std::atomic_bool costDesiredTrajectoriesBufferUpdated_;
  CostDesiredTrajectories costDesiredTrajectoriesBuffer_;
};

}  // namespace ocs2

#include "implementation/MPC_MRT_Interface.h"
