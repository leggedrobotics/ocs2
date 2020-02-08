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

#include <ocs2_comm_interfaces/ocs2_interfaces/MPC_MRT_Interface.h>
#include <ocs2_core/constraint/RelaxedBarrierPenalty.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>

namespace ocs2 {

/**
 * PythonInterface provides a unified interface for all systems
 * to the MPC_MRT_Interface to be used for Python bindings
 *
 * @tparam STATE_DIM Dimension of the state of the system
 * @tparam INPUT_DIM Dimension of the input of the system
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class PythonInterface {
 public:
  using dim_t = ocs2::Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename dim_t::scalar_t;
  using state_vector_t = typename dim_t::state_vector_t;
  using input_vector_t = typename dim_t::input_vector_t;
  using scalar_array_t = typename dim_t::scalar_array_t;
  using state_vector_array_t = typename dim_t::state_vector_array_t;
  using input_vector_array_t = typename dim_t::input_vector_array_t;
  using input_matrix_t = typename dim_t::input_matrix_t;
  using state_matrix_t = typename dim_t::state_matrix_t;
  using state_input_matrix_t = typename dim_t::state_input_matrix_t;
  using input_state_matrix_t = typename dim_t::input_state_matrix_t;
  using input_state_matrix_array_t = typename dim_t::input_state_matrix_array_t;
  using state_matrix_array_t = typename dim_t::state_matrix_array_t;
  using cost_t = CostFunctionBase<STATE_DIM, INPUT_DIM>;
  using dynamic_vector_t = typename dim_t::dynamic_vector_t;
  using dynamic_vector_array_t = typename dim_t::dynamic_vector_array_t;
  using dynamic_matrix_t = typename dim_t::dynamic_matrix_t;
  using input_matrix_array_t = typename dim_t::input_matrix_array_t;

  /**
   * @brief Constructor
   * @param[in] async: Whether or not to run MPC in a separate thread
   */
  explicit PythonInterface(bool async = false);

  //! @brief Destructor
  virtual ~PythonInterface();

  /**
   * @brief initializes the class. This must happen before any other method is called
   * @note init is not called in the constructor because it internally calls pure virtual initRobotInterface
   * @param[in] robotInterface : The Python interface takes clones of the cost, dynamics, and constraints
   * @param[in] mpcPtr : The Python interface takes ownership of the mpcPtr
   */
  void init(const RobotInterface<STATE_DIM, INPUT_DIM>& robotInterface, std::unique_ptr<MPC_BASE<STATE_DIM, INPUT_DIM>> mpcPtr);

  /**
   * @brief resets MPC to its original state
   * @param[in] targetTrajectories: The new target to be optimized for after resetting
   */
  void reset(CostDesiredTrajectories targetTrajectories);

  /**
   * @brief setObservation provides the MPC with a new starting time and state
   * @param[in] t current time
   * @param[in] x current state
   */
  void setObservation(double t, Eigen::Ref<const state_vector_t> x);

  /**
   * @brief setTargetTrajectories
   * @param targetTrajectories
   */
  void setTargetTrajectories(CostDesiredTrajectories targetTrajectories);

  /**
   * @brief run MPC
   * @note This call is blocking in synchronous mode
   */
  void advanceMpc();

  /**
   * @brief Obtain the full MPC solution
   * @param[out] t time array
   * @param[out] x state array
   * @param[out] u input array
   */
  void getMpcSolution(scalar_array_t& t, state_vector_array_t& x, input_vector_array_t& u);

  /**
   * @brief Obtains feedback gain matrix, if the underlying MPC algorithm computes it
   * @param[in] t: Query time
   * @return State-feedback matrix
   */
  input_state_matrix_t getLinearFeedbackGain(scalar_t t);

  /**
   * @brief Access to system dynamics flow map
   * @param[in] t time
   * @param[in] x state
   * @param[in] u input
   * @return time derivative of state (dx/dt)
   */
  state_vector_t computeFlowMap(double t, Eigen::Ref<const state_vector_t> x, Eigen::Ref<const input_vector_t> u);

  /**
   * @brief Sets time, state, input for evaluation of flow map derivatives
   * @param[in] t time
   * @param[in] x state
   * @param[in] u input
   */
  void setFlowMapDerivativeStateAndControl(double t, Eigen::Ref<const state_vector_t> x, Eigen::Ref<const input_vector_t> u);

  /**
   * @brief Access system dynamics state derivative at location set with setFlowMapDerivativeStateAndControl(...)
   * @return state deriative of flow map
   */
  state_matrix_t computeFlowMapDerivativeState();

  /**
   * @brief Access system dynamics input derivative at location set with setFlowMapDerivativeStateAndControl(...)
   * @return input deriative of flow map
   */
  state_input_matrix_t computeFlowMapDerivativeInput();

  /**
   * @brief Access intermediate cost
   * @param[in] t time
   * @param[in] x state
   * @param[in] u input
   * @return Running cost at provided t-x-u
   */
  double getIntermediateCost(double t, Eigen::Ref<const state_vector_t> x, Eigen::Ref<const input_vector_t> u);

  /**
   * @brief Access to state derivative of intermediate cost (L)
   * @param[in] t time
   * @param[in] x state
   * @param[in] u input
   * @return dL/dx at provided t-x-u
   */
  state_vector_t getIntermediateCostDerivativeState(double t, Eigen::Ref<const state_vector_t> x, Eigen::Ref<const input_vector_t> u);

  /**
   * @brief Access to input derivative of intermediate cost (L)
   * @param[in] t time
   * @param[in] x state
   * @param[in] u input
   * @return dL/du at provided t-x-u
   */
  input_vector_t getIntermediateCostDerivativeInput(double t, Eigen::Ref<const state_vector_t> x, Eigen::Ref<const input_vector_t> u);

  /**
   * @brief Access to second input derivative of intermediate cost (L)
   * @param[in] t time
   * @param[in] x state
   * @param[in] u input
   * @return d^2L/du^2 at provided t-x-u
   */
  input_matrix_t getIntermediateCostSecondDerivativeInput(double t, Eigen::Ref<const state_vector_t> x, Eigen::Ref<const input_vector_t> u);

  /**
   * @brief Access the solver's internal value function
   * @param t query time
   * @param x query state
   * @return value function at given t-x
   */
  double getValueFunction(double t, Eigen::Ref<const state_vector_t> x);

  /**
   * @brief Access the solver's internal state derivative of the value function
   * @warning This quantity might only be valid in the vicinity of the optimal trajectory
   * @param[in] t time
   * @param[in] x state
   * @return value function state derivative at given t-x
   */
  state_vector_t getValueFunctionStateDerivative(double t, Eigen::Ref<const state_vector_t> x);

  /**
   * @brief Access state-input constraint value
   * @param[in] t time
   * @param[in] x state
   * @param[in] u input
   * @return The value of the state-input constraint at given t-x-u
   */
  dynamic_vector_t getStateInputConstraint(double t, Eigen::Ref<const state_vector_t> x, Eigen::Ref<const input_vector_t> u);

  /**
   * @brief Access to the input(control) derivative of the state-input constraint
   * @param[in] t time
   * @param[in] x state
   * @param[in] u input
   * @return The input derivative of the state-input constraint at given t-x-u
   */
  dynamic_matrix_t getStateInputConstraintDerivativeControl(double t, Eigen::Ref<const state_vector_t> x,
                                                            Eigen::Ref<const input_vector_t> u);

  /**
   * @brief Access to the lagrangian multiplier of the state-input constraint
   * @param[in] t time
   * @param[in] x state
   * @param[in] u input
   * @return The lagrangian multiplier vector at given t-x
   */
  dynamic_vector_t getStateInputConstraintLagrangian(double t, Eigen::Ref<const state_vector_t> x);

  /**
   * @brief Visualize the time-state-input trajectory
   * @param[in] t Array of times
   * @param[in] x Array of states
   * @param[in] u (Optional) Array of inputs
   * @param[in] speed (Optional) Factor compared to real time playback (>1 ==> slow motion)
   */
  virtual void visualizeTrajectory(const scalar_array_t& t, const state_vector_array_t& x,
                                   const input_vector_array_t& u = input_vector_array_t(), double speed = 1.0) {
    throw std::runtime_error("PythonInterface::visualizeTrajectory must be implemented by robot-specific derived class.");
  }

 private:
  /**
   * @brief runMpcAsync: Worker thread used in asynchronous mode
   */
  void runMpcAsync();

  // Member variables
 protected:
  std::unique_ptr<MPC_BASE<STATE_DIM, INPUT_DIM>> mpcPtr_;
  std::unique_ptr<MPC_MRT_Interface<STATE_DIM, INPUT_DIM>> mpcMrtInterface_;

  std::unique_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM>> dynamics_;
  std::unique_ptr<DerivativesBase<STATE_DIM, INPUT_DIM>> dynamicsDerivatives_;

  std::unique_ptr<ConstraintBase<STATE_DIM, INPUT_DIM>> constraints_;

  std::unique_ptr<cost_t> cost_;
  CostDesiredTrajectories targetTrajectories_;

  std::unique_ptr<PenaltyBase<STATE_DIM, INPUT_DIM>> penalty_;

  // multithreading helper variables
  bool run_mpc_async_;
  std::thread run_mpc_worker_;
  std::mutex run_mpc_mutex_;
  std::condition_variable run_mpc_cv_;
  bool run_mpc_done_;
  std::atomic_bool run_mpc_requested_;
  bool shutdown_requested_;
};

}  // namespace ocs2

#include "implementation/Python_Interface.h"
