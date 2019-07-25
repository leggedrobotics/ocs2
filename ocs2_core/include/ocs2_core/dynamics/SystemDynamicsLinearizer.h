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
    class SystemDynamicsLinearizer
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      typedef DerivativesBase<STATE_DIM, INPUT_DIM> derivatives_system_base_t;
      typedef ControlledSystemBase<STATE_DIM, INPUT_DIM> controlled_system_base_t;
      using scalar_t = typename derivatives_system_base_t::scalar_t;
      using state_vector_t = typename derivatives_system_base_t::state_vector_t;
      using state_matrix_t = typename derivatives_system_base_t::state_matrix_t;
      using input_vector_t = typename derivatives_system_base_t::input_vector_t;
      using state_input_matrix_t = typename derivatives_system_base_t::state_input_matrix_t;

      /**
       * Constructor
       */
      SystemDynamicsLinearizer(const std::shared_ptr<controlled_system_base_t>& nonlinearSystemPtr_,
	  const std::shared_ptr<derivatives_system_base_t>& derivativesSystemPtr_,
	  bool doubleSidedDerivative=true, bool isSecondOrderSystem=false)
	: nonlinearSystemPtr_(nonlinearSystemPtr_),
	derivativesSystemPtr_(derivativesSystemPtr_),
	doubleSidedDerivative_(doubleSidedDerivative),
	isSecondOrderSystem_(isSecondOrderSystem)
    {}

      /**
       * Copy constructor
       *
       * @param [in] other: Instance of the other class.
       */
      SystemDynamicsLinearizer(const SystemDynamicsLinearizer& other) = default;
        //
	// :
	// nonlinearSystemPtr_(other.nonlinearSystemPtr_),
	// derivativesSystemPtr_(other.derivativesSystemPtr_),
	// doubleSidedDerivative_(other.doubleSidedDerivative_),
	// isSecondOrderSystem_(other.isSecondOrderSystem_)
	// {}

      /**
       * operator=
       *
       * @param [in] other: Instance of the other class.
       * @return SystemDynamicsLinearizer&:
       */
      SystemDynamicsLinearizer& operator=(const SystemDynamicsLinearizer&other) // = default;
      {
        nonlinearSystemPtr_ = other.nonlinearSystemPtr_;
        derivativesSystemPtr_ = other.derivativesSystemPtr_;
        doubleSidedDerivative_ = other.doubleSidedDerivative_;
        isSecondOrderSystem_ = other.isSecondOrderSystem_;
      }


      /**
       * Default destructor
       */
      virtual ~SystemDynamicsLinearizer()= default;

      void setControlSystem( const std::shared_ptr<controlled_system_base_t>& nonlinearSystemPtr)
      {
	nonlinearSystemPtr_=nonlinearSystemPtr;
      }

      void setDerivativesSystem(const std::shared_ptr<derivatives_system_base_t>& derivativesSystemPtr)
      {
	derivativesSystemPtr_=derivativesSystemPtr;
      }

      void setSystems(  const std::shared_ptr<controlled_system_base_t>& nonlinearSystemPtr, const std::shared_ptr<derivatives_system_base_t>& derivativesSystemPtr ){
	setControlSystem(nonlinearSystemPtr);
	setDerivativesSystem(derivativesSystemPtr);
      }


      /**
       * The A matrix at a given operating point for the linearized system,
       * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
       *
       * @param [out] A: \f$ A(t) \f$ matrix.
       */
      void getDerivativesState(state_matrix_t& A, const scalar_t &t, const state_vector_t &x, const input_vector_t &u) {

	for (size_t i=0; i<STATE_DIM; i++)  {

	  // inspired from http://en.wikipedia.org/wiki/Numerical_differentiation#Practical_considerations_using_floating_point_arithmetic
	  scalar_t h = eps_ * std::max(fabs(x(i)), 1.0);

	  state_vector_t xPlusPerturbed = x;
	  xPlusPerturbed(i) += h;

	  // get evaluation of f(x,u)
	  state_vector_t fPlusPerturbed;
	  nonlinearSystemPtr_->computeFlowMap(t, xPlusPerturbed, u, fPlusPerturbed);

	  if (doubleSidedDerivative_)  {

	    state_vector_t xMinusPerturbed = x;
	    xMinusPerturbed(i) -= h;

	    state_vector_t fMinusPerturbed;
	    nonlinearSystemPtr_->computeFlowMap(t, xMinusPerturbed, u, fMinusPerturbed);

	    if(isSecondOrderSystem_)  {
	      A.template topLeftCorner<STATE_DIM/2, STATE_DIM/2>().setZero();
	      A.template topRightCorner<STATE_DIM/2, STATE_DIM/2>().setIdentity();
	      A.template block<STATE_DIM/2,1>(STATE_DIM/2,i) = (fPlusPerturbed.template tail<STATE_DIM/2>() - fMinusPerturbed.template tail<STATE_DIM/2>()) / (2.0*h);
	    }
	    else {
	      A.col(i) = (fPlusPerturbed - fMinusPerturbed) / (2.0*h);
	    }
	  }
	  else  {
	    if(isSecondOrderSystem_)  {
	      A.template topLeftCorner<STATE_DIM/2, STATE_DIM/2>().setZero();
	      A.template topRightCorner<STATE_DIM/2, STATE_DIM/2>().setIdentity();
	      A.template block<STATE_DIM/2,1>(STATE_DIM/2,i) = (fPlusPerturbed.template tail<STATE_DIM/2>() - f_.template tail<STATE_DIM/2>()) / h;
	    }
	    else {
	      A.col(i) = (fPlusPerturbed - f_) / h;
	    }
	  }
	}  // end of i loop

      }

      /**
       * The B matrix at a given operating point for the linearized system,
       * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
       *
       * @param [out] B: \f$ B(t) \f$ matrix.
       */
      void getDerivativesControl(state_input_matrix_t& B, const scalar_t &t, const state_vector_t &x, const input_vector_t &u) {

	Eigen::Matrix<double, STATE_DIM, INPUT_DIM> tempB;

	for (size_t i=0; i<INPUT_DIM; i++) {

	  //			std::cin.get();

	  // inspired from http://en.wikipedia.org/wiki/Numerical_differentiation#Practical_considerations_using_floating_pointarithmetic
	  double h = eps_ * std::max(fabs(u(i)), 1.0);

	  input_vector_t uPlusPerturbed = u;
	  uPlusPerturbed(i) += h;

	  // get evaluation of f(x,u)
	  state_vector_t fPlusPerturbed;
	  nonlinearSystemPtr_->computeFlowMap(t, x, uPlusPerturbed, fPlusPerturbed);

	  if (doubleSidedDerivative_)  {

	    input_vector_t uMinusPerturbed = u;
	    uMinusPerturbed(i) -= h;

	    state_vector_t fMinusPerturbed;
	    nonlinearSystemPtr_->computeFlowMap(t, x, uMinusPerturbed, fMinusPerturbed);

	    if(isSecondOrderSystem_)  {
	      tempB.template topRows<STATE_DIM/2>().setZero();
	      tempB.template block<STATE_DIM/2,1>(STATE_DIM/2,i) = (fPlusPerturbed.template tail<STATE_DIM/2>() - fMinusPerturbed.template tail<STATE_DIM/2>()) / (2.0*h);
	    }
	    else {
	      tempB.col(i) = (fPlusPerturbed - fMinusPerturbed) / (2.0*h);

	      //					std::cout << ">>>> The " << i << " element out of " << INPUT_DIM << std::endl;
	      //					std::cout << "u+ :" << uPlusPerturbed.transpose() << std::endl;
	      //					std::cout << "f+ :" << fPlusPerturbed.transpose() << std::endl;
	      //					std::cout << "u- :" << uMinusPerturbed.transpose() << std::endl;
	      //					std::cout << "f- :" << fMinusPerturbed.transpose() << std::endl << std::endl;
	    }
	  }
	  else {
	    if(isSecondOrderSystem_)  {
	      tempB.template topRows<STATE_DIM/2>().setZero();
	      tempB.template block<STATE_DIM/2,1>(STATE_DIM/2,i) = (fPlusPerturbed.template tail<STATE_DIM/2>() - f_.template tail<STATE_DIM/2>()) / h;
	    }
	    else {
	      tempB.col(i) = (fPlusPerturbed - f_) / h;
	    }
	  }
	}  // end of i loop

	//B = nonlinearSystemPtr_->computeOutputStateDerivative(t, x, u) * tempB;
	B = tempB;
      }

      /**
       * Checks if the linearized dynamics using a finite difference method fall within the tolerance.
       *
       * @param[in] controlledSystemBase
       * @param[in] derivativesBase
       * @param[in] t time to check at
       * @param[in] x state to check at
       * @param[in] u control to check at
       * @param[out] error  the numerical finite difference method to linearized dynamic's error
       * @return whether the error is less than the tolerance specified in @property tolerance_
       */
      bool derivativeChecker(scalar_t t, const state_vector_t &x, const input_vector_t &u, state_matrix_t &A_error, state_input_matrix_t &B_error, scalar_t tolerance)
      {
        derivativesSystemPtr_->setCurrentStateAndControl(t, x, u);

	state_matrix_t A;
	state_input_matrix_t B;
	derivativesSystemPtr_->getFlowMapDerivativeState(A);
	derivativesSystemPtr_->getFlowMapDerivativeInput(B);

	// Finite Difference Methods
	getDerivativesState(A_error, t, x, u);
	A_error -= A;
	getDerivativesControl(B_error, t, x, u);
	B_error -= B;
	//A_error.cwiseAbs();
	//B_error.cwiseAbs();
	return tolerance > std::fmax(A_error.template lpNorm<Eigen::Infinity>(), B_error.template lpNorm<Eigen::Infinity>());
      }


      /**
       * Returns pointer to the class.
       *
       * @return A raw pointer to the class.
       */
      //SystemDynamicsLinearizer<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>* clone() const override {
      //return new SystemDynamicsLinearizer<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>(*this);
      //}


    private:
      const scalar_t eps_= sqrt(Eigen::NumTraits<scalar_t>::epsilon());
      typename derivatives_system_base_t::Ptr derivativesSystemPtr_;
      typename controlled_system_base_t::Ptr nonlinearSystemPtr_;
      bool doubleSidedDerivative_;
      bool isSecondOrderSystem_;

      state_vector_t f_;

  };

}  // namespace ocs2

#endif /* SYSTEMDYNAMICSLINEARIZER_H_ */
