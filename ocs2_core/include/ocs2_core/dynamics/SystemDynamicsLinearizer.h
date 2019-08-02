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

#ifndef SYSTEMDYNAMICSLINEARIZER_OCS2_H_
#define SYSTEMDYNAMICSLINEARIZER_OCS2_H_

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <memory>

#include "ocs2_core/dynamics/ControlledSystemBase.h"
#include "ocs2_core/dynamics/DerivativesBase.h"
#include "ocs2_core/automatic_differentiation/FiniteDifferenceMethods.h"

namespace ocs2 {

  /**
   * A class for linearizing system dynamics. The linearized system dynamics is defined as: \n
   *
   * - Linearized system:                        \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$ \n
   *
   * @tparam STATE_DIM: Dimension of the state space.
   * @tparam INPUT_DIM: Dimension of the control input space.
   */
  template <size_t STATE_DIM, size_t INPUT_DIM>
    class SystemDynamicsLinearizer : public DerivativesBase<STATE_DIM, INPUT_DIM>, public FiniteDifferenceMethods<ControlledSystemBase<STATE_DIM, INPUT_DIM>>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      typedef DerivativesBase<STATE_DIM, INPUT_DIM> Base;
      typedef ControlledSystemBase<STATE_DIM, INPUT_DIM> controlled_system_base_t;
      typedef FiniteDifferenceMethods<controlled_system_base_t> finite_difference_methods_t;
      using scalar_t = typename Base::scalar_t;
      using state_vector_t = typename Base::state_vector_t;
      using state_matrix_t = typename Base::state_matrix_t;
      using input_vector_t = typename Base::input_vector_t;
      using state_input_matrix_t = typename Base::state_input_matrix_t;

      /**
       * Constructor
       */
      SystemDynamicsLinearizer(const std::shared_ptr<controlled_system_base_t>& nonlinearSystemPtr,
	  bool doubleSidedDerivative=true, bool isSecondOrderSystem=false)
	: finite_difference_methods_t(Eigen::NumTraits<scalar_t>::epsilon(),
	    std::numeric_limits<scalar_t>::infinity(),
	    nonlinearSystemPtr,
	    doubleSidedDerivative, isSecondOrderSystem)    {}

      /**
       * Copy constructor
       *
       * @param [in] other: Instance of the other class.
       */
      SystemDynamicsLinearizer(const SystemDynamicsLinearizer& other) = default;

      /**
       * operator=
       *
       * @param [in] other: Instance of the other class.
       * @return SystemDynamicsLinearizer&:
       */
      SystemDynamicsLinearizer& operator=(const SystemDynamicsLinearizer& other)  = default;


      /**
       * Default destructor
       */
      virtual ~SystemDynamicsLinearizer()= default;


      /**
       * The A matrix at a given operating point for the linearized system,
       * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
       *
       * @param [out] A: \f$ A(t) \f$ matrix.
       */
      void getFlowMapDerivativeState(state_matrix_t& A) override {
	return this->finiteDifferenceDerivativeState(this->x_, this->u_, this->t_, A);
      }

      /**
       * The B matrix at a given operating point for the linearized system,
       * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
       *
       * @param [out] B: \f$ B(t) \f$ matrix.
       */
      void getFlowMapDerivativeInput(state_input_matrix_t& B) override {
	return this->finiteDifferenceDerivativeInput(this->u_, this->x_, this->t_, B);
      }

      // It is not logical to call the function from the SystemDynamicsLinearizer
      static bool derivativeChecker(const std::shared_ptr<controlled_system_base_t>& nonlinearSystemPtr, 
	  const std::shared_ptr<Base>& derivativesBasePtr,
	  const scalar_t t, const state_vector_t &x, const input_vector_t &u,
	  state_matrix_t &A_error, state_input_matrix_t &B_error,
	  scalar_t tolerance) = delete;

      /**
       * Returns pointer to the class.
       *
       * @return A raw pointer to the class.
       */
      SystemDynamicsLinearizer<STATE_DIM, INPUT_DIM>* clone() const override {
      return new SystemDynamicsLinearizer<STATE_DIM, INPUT_DIM>(*this);
      }


    private:

      // typename controlled_system_base_t::Ptr nonlinearSystemPtr_;
      // bool doubleSidedDerivative_;
      // bool isSecondOrderSystem_;
      // state_vector_t f_;
      // friend class FiniteDifferenceMethods;

};

}  // namespace ocs2

#endif /* SYSTEMDYNAMICSLINEARIZER_H_ */
