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

#include <ocs2_ddp/SLQ.h>

namespace ocs2 {

/**
 * NumGDDP class for computing numerical gradient of the cost function w.r.t. event times.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class NumGDDP : public SLQ<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef SLQ<STATE_DIM, INPUT_DIM> BASE;

  using typename BASE::constraint_base_t;
  using typename BASE::controlled_system_base_t;
  using typename BASE::cost_function_base_t;
  using typename BASE::derivatives_base_t;
  using typename BASE::dynamic_vector_t;
  using typename BASE::input_vector_t;
  using typename BASE::operating_trajectories_base_t;
  using typename BASE::scalar_array_t;
  using typename BASE::scalar_t;
  using typename BASE::state_vector_t;

  /**
   * Constructor.
   *
   * @param [in] systemDynamicsPtr: The system dynamics which possibly includes some subsystems.
   * @param [in] systemDerivativesPtr: The system dynamics derivatives for subsystems of the system.
   * @param [in] systemConstraintsPtr: The system constraint function and its derivatives for subsystems.
   * @param [in] costFunctionPtr: The cost function (intermediate and terminal costs) and its derivatives for subsystems.
   * @param [in] operatingTrajectoriesPtr: The operating trajectories of system which will be used for initialization of SLQ.
   * @param [in] settings: Structure containing the settings for the SLQ algorithm.
   * @param [in] logicRulesPtr: The logic rules used for implementing mixed logical dynamical systems.
   * @param [in] heuristicsFunctionPtr: Heuristic function used in the infinite time optimal control formulation. If it is not
   * defined, we will use the terminal cost function defined in costFunctionPtr.
   */
  NumGDDP(const controlled_system_base_t* systemDynamicsPtr, const derivatives_base_t* systemDerivativesPtr,
          const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr,
          const operating_trajectories_base_t* operatingTrajectoriesPtr, const SLQ_Settings& settings = SLQ_Settings(),
          const LOGIC_RULES_T* logicRulesPtr = nullptr, const cost_function_base_t* heuristicsFunctionPtr = nullptr);

  /**
   * Default destructor.
   */
  virtual ~NumGDDP() = default;

  /**
   * Calculates the cost function's derivatives w.r.t. event times.
   *
   * @param [out] costFunctionDerivative: cost function's derivatives w.r.t. event times.
   */
  template <typename Derived>
  void getCostFuntionDerivative(Eigen::MatrixBase<Derived> const& costFunctionDerivative) const;

  /**
   * Runs the NumGDDP to compute the gradient of the cost function w.r.t. the event times.
   *
   * @param [in] initTime: The initial time.
   * @param [in] initState: The initial state.
   * @param [in] finalTime: The final time.
   * @param [in] partitioningTimes: The partitioning times between subsystems.
   * @param [in] eventTimes: The event times vector.
   */
  void run(const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime, const scalar_array_t& partitioningTimes,
           const scalar_array_t& eventTimes);

 protected:
  /**
   * Sets the event times to the solver.
   *
   * @param [in] eventTimes: The event times vector.
   */
  void setSolverEventTime(const scalar_array_t& eventTimes);

  /***********
   * Variables
   **********/
  scalar_array_t eventTimes_;
  size_t numEventTimes_ = 0;
  scalar_t eps_;

  size_t activeEventTimeBeginIndex_;
  size_t activeEventTimeEndIndex_;

  dynamic_vector_t nominalCostFuntionDerivative_;
};

}  // namespace ocs2

#include "implementation/NumGDDP.h"
