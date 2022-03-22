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

#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_core/penalties/penalties/PenaltyBase.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>

namespace ocs2 {

/**
 * PythonInterface provides a unified interface for all systems
 * to the MPC_MRT_Interface to be used for Python bindings
 */
class PythonInterface {
 protected:
  /** Constructor */
  PythonInterface() = default;

  /**
   * Initialize python bindings
   * @note This should be called from derived class constructor.
   * @param [in] robot: Robot interface.
   * @param [in] mpcPtr: The Python interface takes ownership of the mpcPtr
   */
  void init(const RobotInterface& robot, std::unique_ptr<MPC_BASE> mpcPtr);

 public:
  /** Destructor */
  virtual ~PythonInterface() = default;

  /**
   * @brief Get the state dimension of the dynamics system.
   *
   * @return The number of states in the system.
   */
  int getStateDim() const { return stateDim_; }

  /**
   * @brief Get the input dimension of the dynamics system.
   *
   * @return The number of inputs in the system.
   */
  int getInputDim() const { return inputDim_; }

  /**
   * @brief resets MPC to its original state
   * @param[in] targetTrajectories: The new target to be optimized for after resetting
   */
  void reset(TargetTrajectories targetTrajectories);

  /**
   * @brief setObservation provides the MPC with a new starting time and state
   * @param[in] t current time
   * @param[in] x current state
   * @param[in] u current input
   */
  void setObservation(scalar_t t, Eigen::Ref<const vector_t> x, Eigen::Ref<const vector_t> u);

  /**
   * @brief setTargetTrajectories
   * @param targetTrajectories
   */
  void setTargetTrajectories(TargetTrajectories targetTrajectories);

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
  void getMpcSolution(scalar_array_t& t, vector_array_t& x, vector_array_t& u);

  /**
   * @brief Obtains feedback gain matrix, if the underlying MPC algorithm computes it
   * @param[in] t: Query time
   * @return State-feedback matrix
   */
  matrix_t getLinearFeedbackGain(scalar_t t);

  /** System dynamics */
  vector_t flowMap(scalar_t t, Eigen::Ref<const vector_t> x, Eigen::Ref<const vector_t> u);

  /** System dynamics linearization */
  VectorFunctionLinearApproximation flowMapLinearApproximation(scalar_t t, Eigen::Ref<const vector_t> x, Eigen::Ref<const vector_t> u);

  /** Cost function with added penalty term */
  scalar_t cost(scalar_t t, Eigen::Ref<const vector_t> x, Eigen::Ref<const vector_t> u);

  /** Cost function quadratic approximation with added penalty term */
  ScalarFunctionQuadraticApproximation costQuadraticApproximation(scalar_t t, Eigen::Ref<const vector_t> x, Eigen::Ref<const vector_t> u);

  /**
   * The solver's internal value function
   * @param t query time
   * @param x query state
   * @return value function at given t-x
   */
  scalar_t valueFunction(scalar_t t, Eigen::Ref<const vector_t> x);

  /**
   * The solver's internal value function derivative w.r.t. state.
   * @warning This quantity might only be valid in the vicinity of the optimal trajectory
   * @param[in] t time
   * @param[in] x state
   * @return value function state derivative at given t-x
   */
  vector_t valueFunctionStateDerivative(scalar_t t, Eigen::Ref<const vector_t> x);

  /**
   * @brief Access state-input constraint value
   * @param[in] t time
   * @param[in] x state
   * @param[in] u input
   * @return The value of the state-input constraint at given t-x-u
   */
  vector_t stateInputEqualityConstraint(scalar_t t, Eigen::Ref<const vector_t> x, Eigen::Ref<const vector_t> u);

  /**
   * @brief Access to the input(control) derivative of the state-input constraint
   * @param[in] t time
   * @param[in] x state
   * @param[in] u input
   * @return The input derivative of the state-input constraint at given t-x-u
   */
  VectorFunctionLinearApproximation stateInputEqualityConstraintLinearApproximation(scalar_t t, Eigen::Ref<const vector_t> x,
                                                                                    Eigen::Ref<const vector_t> u);
  /**
   * @brief Access to the lagrangian multiplier of the state-input constraint
   * @param[in] t time
   * @param[in] x state
   * @param[in] u input
   * @return The lagrangian multiplier vector at given t-x
   */
  vector_t stateInputEqualityConstraintLagrangian(scalar_t t, Eigen::Ref<const vector_t> x, Eigen::Ref<const vector_t> u);

  /**
   * @brief Visualize the time-state-input trajectory
   * @param[in] t Array of times
   * @param[in] x Array of states
   * @param[in] u (Optional) Array of inputs
   * @param[in] speed (Optional) Factor compared to real time playback (>1 ==> slow motion)
   */
  virtual void visualizeTrajectory(const scalar_array_t& t, const vector_array_t& x, const vector_array_t& u = vector_array_t(),
                                   scalar_t speed = 1.0) {
    throw std::runtime_error("PythonInterface::visualizeTrajectory must be implemented by robot-specific derived class.");
  }

 protected:
  std::unique_ptr<PenaltyBase> penalty_;

  int stateDim_ = -1;  // -1 indicates that it is not initialized
  int inputDim_ = -1;  // -1 indicates that it is not initialized

 private:
  std::unique_ptr<MPC_BASE> mpcPtr_;
  std::unique_ptr<MPC_MRT_Interface> mpcMrtInterface_;

  TargetTrajectories targetTrajectories_;
  OptimalControlProblem problem_;
};

}  // namespace ocs2
