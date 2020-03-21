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

#ifndef SYSTEMDYNAMICSBASE_OCS2_H_
#define SYSTEMDYNAMICSBASE_OCS2_H_

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/dynamics/ControlledSystemBase.h"
#include "ocs2_core/dynamics/DerivativesBase.h"

namespace ocs2 {

/**
 * The system dynamics interface.
 * The linearized system flow map is defined as: \n
 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$ \n
 * The linearized system jump map is defined as: \n
 * \f$ x^+ = G \delta x + H \delta u \f$ \n
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM, size_t NUM_MODES = 1>
class SystemDynamicsBase : public DerivativesBase<STATE_DIM, INPUT_DIM>, public ControlledSystemBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<SystemDynamicsBase<STATE_DIM, INPUT_DIM> >;
  using ConstPtr = std::shared_ptr<const SystemDynamicsBase<STATE_DIM, INPUT_DIM> >;

  using DEV_BASE = DerivativesBase<STATE_DIM, INPUT_DIM>;
  using DYN_BASE = ControlledSystemBase<STATE_DIM, INPUT_DIM>;

  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using state_matrix_t = typename DIMENSIONS::state_matrix_t;
  using state_input_matrix_t = typename DIMENSIONS::state_input_matrix_t;
  using dynamic_vector_t = typename DIMENSIONS::dynamic_vector_t;
  using dynamic_state_matrix_t = typename DIMENSIONS::dynamic_state_matrix_t;
  using dynamic_input_matrix_t = typename DIMENSIONS::dynamic_input_matrix_t;

  /**
   * Default constructor
   */
  SystemDynamicsBase() {
    if (NUM_MODES == 0) {
      throw std::runtime_error("The system should have at least one mode.");
    }
  }

  /**
   * Copy constructor
   */
  SystemDynamicsBase(const SystemDynamicsBase& rhs) = default;

  /**
   * Default destructor
   */
  ~SystemDynamicsBase() override = default;

  /**
   * Returns pointer to the class.
   *
   * @return A raw pointer to the class.
   */
  virtual SystemDynamicsBase<STATE_DIM, INPUT_DIM>* clone() const = 0;

  /**
   * Gets the number of the modes in the hybrid system.
   * The minimum is one.
   *
   * @return Number of the modes in the hybrid system.
   */
  size_t getNumModes() const { return NUM_MODES; }
};

}  // namespace ocs2

#endif /* SYSTEMDYNAMICSBASE_OCS2_H_ */
