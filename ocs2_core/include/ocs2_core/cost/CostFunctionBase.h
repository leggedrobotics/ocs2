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

#include <memory>

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/cost/CostDesiredTrajectories.h"
#include "ocs2_core/misc/LinearInterpolation.h"

namespace ocs2 {

/**
 * Cost Function Base.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class CostFunctionBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum { state_dim_ = STATE_DIM, input_dim_ = INPUT_DIM };

  using Ptr = std::shared_ptr<CostFunctionBase<STATE_DIM, INPUT_DIM> >;
  using ConstPtr = std::shared_ptr<const CostFunctionBase<STATE_DIM, INPUT_DIM> >;

  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using state_matrix_t = typename DIMENSIONS::state_matrix_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
  using input_matrix_t = typename DIMENSIONS::input_matrix_t;
  using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t;
  using state_input_matrix_t = typename DIMENSIONS::state_input_matrix_t;
  using dynamic_vector_t = typename DIMENSIONS::dynamic_vector_t;
  using dynamic_vector_array_t = typename DIMENSIONS::dynamic_vector_array_t;

  /**
   * Default constructor
   */
  CostFunctionBase() : costDesiredTrajectoriesPtr_(nullptr) {}

  /**
   * Copy constructor
   */
  CostFunctionBase(const CostFunctionBase& rhs) : CostFunctionBase() {}

  /**
   * Default destructor
   */
  virtual ~CostFunctionBase() = default;

  /**
   * Sets the desired state and input trajectories used in the cost function.
   *
   * @param [in] CostDesiredTrajectoriesPtr: A cost pointer to desired trajectories.
   */
  virtual void setCostDesiredTrajectoriesPtr(const CostDesiredTrajectories* costDesiredTrajectoriesPtr) {
    costDesiredTrajectoriesPtr_ = costDesiredTrajectoriesPtr;
  }

  /**
   * Returns pointer to the class.
   *
   * @return A raw pointer to the class.
   */
  virtual CostFunctionBase<STATE_DIM, INPUT_DIM>* clone() const = 0;

  /**
   * Sets the current time, state, and control input
   *
   * @param [in] t: Current time
   * @param [in] x: Current state vector
   * @param [in] u: Current input vector
   */
  virtual void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) {
    t_ = t;
    x_ = x;
    u_ = u;
  }

  /**
   * Get the intermediate cost.
   *
   * @param [out] L: The intermediate cost value.
   */
  virtual void getIntermediateCost(scalar_t& L) = 0;

  /**
   * Get the time derivative of the intermediate cost.
   *
   * @param [out] dLdt: The time derivative of intermediate cost.
   */
  virtual void getIntermediateCostDerivativeTime(scalar_t& dLdt) { dLdt = 0; }

  /**
   * Get the state derivative of the intermediate cost.
   *
   * @param [out] dLdx: First order derivative of the intermediate cost with respect to state vector.
   */
  virtual void getIntermediateCostDerivativeState(state_vector_t& dLdx) = 0;

  /**
   * Get state second order derivative of the intermediate cost.
   *
   * @param [out] dLdxx: Second order derivative of the intermediate cost with respect to state vector.
   */
  virtual void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) = 0;

  /**
   * Get control input derivative of the intermediate cost.
   *
   * @param [out] dLdu: First order derivative of the intermediate cost with respect to input vector.
   */
  virtual void getIntermediateCostDerivativeInput(input_vector_t& dLdu) = 0;

  /**
   * Get control input second derivative of the intermediate cost.
   *
   * @param [out] dLduu: Second order derivative of the intermediate cost with respect to input vector.
   */
  virtual void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) = 0;

  /**
   * Get the input-state derivative of the intermediate cost.
   *
   * @param [out] dLdux: Second order derivative of the intermediate cost with respect to input vector and state.
   */
  virtual void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdux) = 0;

  /**
   * Get the terminal cost.
   *
   * @param [out] Phi: The final cost value.
   */
  virtual void getTerminalCost(scalar_t& Phi) = 0;

  /**
   * Get the time derivative of terminal cost.
   *
   * @param [out] dPhidt: The time derivative of terminal cost.
   */
  virtual void getTerminalCostDerivativeTime(scalar_t& dPhidt) { dPhidt = 0; }

  /**
   * Get the terminal cost state derivative of the terminal cost.
   *
   * @param [out] dPhidx: First order final cost derivative with respect to state vector.
   */
  virtual void getTerminalCostDerivativeState(state_vector_t& dPhidx) = 0;

  /**
   * Get the terminal cost state second derivative of the terminal cost.
   *
   * @param [out] dPhidxx: Second order final cost derivative with respect to state vector.
   */
  virtual void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) = 0;

 protected:
  /*
   * Variables
   */
  const CostDesiredTrajectories* costDesiredTrajectoriesPtr_;

  scalar_t t_;
  state_vector_t x_;
  input_vector_t u_;
};

}  // namespace ocs2
