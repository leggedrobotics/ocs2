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

#ifndef CONTROLLEDSYSTEMBASE_OCS2_H_
#define CONTROLLEDSYSTEMBASE_OCS2_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <cstring>
#include <vector>

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/control/ControllerBase.h"
#include "ocs2_core/integration/OdeBase.h"

namespace ocs2 {

/**
 * The base class for non-autonomous system dynamics.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class ControlledSystemBase : public OdeBase<STATE_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> >;
  using ConstPtr = std::shared_ptr<const ControlledSystemBase<STATE_DIM, INPUT_DIM> >;

  using BASE = OdeBase<STATE_DIM>;

  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
  using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t;
  using input_state_matrix_array_t = typename DIMENSIONS::input_state_matrix_array_t;
  using constraint1_vector_t = typename DIMENSIONS::constraint1_vector_t;
  using constraint2_vector_t = typename DIMENSIONS::constraint2_vector_t;
  using dynamic_vector_t = typename DIMENSIONS::dynamic_vector_t;

  using controller_t = ControllerBase<STATE_DIM, INPUT_DIM>;

  /**
   * Default constructor.
   */
  ControlledSystemBase() : BASE(), controllerPtr_(nullptr) {}

  /**
   * Copy constructor.
   */
  ControlledSystemBase(const ControlledSystemBase& rhs) : ControlledSystemBase() { setController(rhs.controllerPtr()); }

  /**
   * Default destructor.
   */
  virtual ~ControlledSystemBase() = default;

  /**
   * Resets the internal classes.
   */
  virtual void reset() { controllerPtr_ = nullptr; }

  /**
   * Sets the control policy using the controller class.
   *
   * @param [in] controllerPtr: A pointer to the control policy.
   */
  void setController(controller_t* controllerPtr) { controllerPtr_ = controllerPtr; }

  /**
   * Computes derivative of the autonomous system dynamics with the given control policy.
   *
   * @param [in] t: Current time.
   * @param [in] x: Current state.
   * @param [out] dxdt: Current state time derivative.
   */
  void computeFlowMap(const scalar_t& t, const state_vector_t& x, state_vector_t& dxdt) override {
    BASE::numFunctionCalls_++;
    input_vector_t u = controllerPtr_->computeInput(t, x);
    computeFlowMap(t, x, u, dxdt);
  }

  /**
   * Returns pointer to the class.
   *
   * @return A raw pointer to the class.
   */
  virtual ControlledSystemBase<STATE_DIM, INPUT_DIM>* clone() const = 0;

  /**
   * Computes derivative of the autonomous system dynamics.
   *
   * @param [in] t: Current time.
   * @param [in] x: Current state.
   * @param [in] u: Current input.
   * @param [out] dxdt: Current state time derivative.
   */
  virtual void computeFlowMap(const scalar_t& t, const state_vector_t& x, const input_vector_t& u, state_vector_t& dxdt) = 0;

  /**
   * Returns the controller pointer.
   *
   * @return A pointer to controller.
   */
  controller_t* controllerPtr() const { return controllerPtr_; }

 private:
  controller_t* controllerPtr_;  //! pointer to controller
};

}  // namespace ocs2

#endif
