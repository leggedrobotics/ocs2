/******************************************************************************
  Copyright (c) 2019, Oliver Harley. All rights reserved.

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

#ifndef FINITEDIFFERENCEMETHODS_H
#define FINITEDIFFERENCEMETHODS_H

#include <random>

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/dynamics/ControlledSystemBase.h"
#include "ocs2_core/dynamics/DerivativesBase.h"
#include "ocs2_core/dynamics/SystemDynamicsLinearizer.h"

namespace ocs2 {

/**
 * OCS2 Derivative Check Class
 * @tparam class ControlledSystemBase
 */
template <class ControlledSystemBase>
class FiniteDifferenceMethods {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr size_t STATE_DIM = ControlledSystemBase::DIMENSIONS::STATE_DIM_;
  static constexpr size_t INPUT_DIM = ControlledSystemBase::DIMENSIONS::INPUT_DIM_;

  using finite_difference_method_t = FiniteDifferenceMethods<ControlledSystemBase>;
  using controlled_system_base_t = ControlledSystemBase;
  using derivatives_base_t = DerivativesBase<STATE_DIM, INPUT_DIM>;

  using scalar_t = typename derivatives_base_t::scalar_t;
  using state_vector_t = typename derivatives_base_t::state_vector_t;
  using input_vector_t = typename derivatives_base_t::input_vector_t;
  using state_matrix_t = typename derivatives_base_t::state_matrix_t;
  using state_input_matrix_t = typename derivatives_base_t::state_input_matrix_t;

 public:
  explicit FiniteDifferenceMethods(std::shared_ptr<controlled_system_base_t> controlledSystem,
                                   scalar_t eps = Eigen::NumTraits<scalar_t>::epsilon(),
                                   scalar_t tolerance = sqrt(Eigen::NumTraits<scalar_t>::epsilon()), bool doubleSidedDerivative = true,
                                   bool isSecondOrderSystem = false)
      : eps_(eps),
        tolerance_(tolerance),
        controlledSystem_(std::move(controlledSystem)),
        doubleSidedDerivative_(doubleSidedDerivative),
        isSecondOrderSystem_(isSecondOrderSystem) {
    if (!controlledSystem_) {
      throw std::invalid_argument("The passed controlled system cannot be null");
    }
  };

  /**
   * Copy constructor
   *
   * @param [in] other: Instance of the other class.
   */
  FiniteDifferenceMethods(const FiniteDifferenceMethods& other)
      : controlledSystem_(other.controlledSystem_->clone()),
        eps_(other.eps_),
        tolerance_(other.tolerance_),
        doubleSidedDerivative_(other.doubleSidedDerivative_),
        isSecondOrderSystem_(other.isSecondOrderSystem_){};

  FiniteDifferenceMethods& operator=(const FiniteDifferenceMethods& other) {
    eps_ = other.eps_;
    controlledSystem_(other.controlledSystem_.clone());
    doubleSidedDerivative_ = other.doubleSidedDerivative_;
    isSecondOrderSystem_ = other.isSecondOrderSystem_;
  }

  static void finiteDifferenceDerivativeState(controlled_system_base_t& system, scalar_t eps, bool doubleSidedDerivative,
                                              bool isSecondOrderSystem, scalar_t t, const state_vector_t& x, const input_vector_t& u,
                                              state_matrix_t& A) {
    // evaluation of f0 = f(x0,u0)
    state_vector_t f;
    system.computeFlowMap(t, x, u, f);

    for (size_t i = 0; i < STATE_DIM; ++i) {
      // inspired from
      // http://en.wikipedia.org/wiki/Numerical_differentiation#Practical_considerations_using_floating_point_arithmetic
      scalar_t h = eps * std::max(fabs(x(i)), 1.0);

      state_vector_t xPlusPerturbed = x;
      xPlusPerturbed(i) += h;

      // get evaluation of f(x,u)
      state_vector_t fPlusPerturbed;
      system.computeFlowMap(t, xPlusPerturbed, u, fPlusPerturbed);

      if (doubleSidedDerivative) {
        state_vector_t xMinusPerturbed = x;
        xMinusPerturbed(i) -= h;

        state_vector_t fMinusPerturbed;
        system.computeFlowMap(t, xMinusPerturbed, u, fMinusPerturbed);

        if (isSecondOrderSystem) {
          std::cout << STATE_DIM;
          A.template topLeftCorner<STATE_DIM / 2, STATE_DIM / 2>().setZero();
          A.template topRightCorner<STATE_DIM / 2, STATE_DIM / 2>().setIdentity();
          A.template block<STATE_DIM / 2, 1>(STATE_DIM / 2, i) =
              (fPlusPerturbed.template tail<STATE_DIM / 2>() - fMinusPerturbed.template tail<STATE_DIM / 2>()) / (2.0 * h);
        } else {
          A.col(i) = (fPlusPerturbed - fMinusPerturbed) / (2.0 * h);
        }
      } else {
        if (isSecondOrderSystem) {
          A.template topLeftCorner<STATE_DIM / 2, STATE_DIM / 2>().setZero();
          A.template topRightCorner<STATE_DIM / 2, STATE_DIM / 2>().setIdentity();
          A.template block<STATE_DIM / 2, 1>(STATE_DIM / 2, i) =
              (fPlusPerturbed.template tail<STATE_DIM / 2>() - f.template tail<STATE_DIM / 2>()) / h;
        } else {
          A.col(i) = (fPlusPerturbed - f) / h;
        }
      }
    }  // end of i loop
  }

  void finiteDifferenceDerivativeState(scalar_t t, const state_vector_t& x, const input_vector_t& u, state_matrix_t& A) {
    return finiteDifferenceDerivativeState(*controlledSystem_, eps_, doubleSidedDerivative_, isSecondOrderSystem_, t, x, u, A);
  }

  static void finiteDifferenceDerivativeInput(controlled_system_base_t& system, scalar_t eps, bool doubleSidedDerivative,
                                              bool isSecondOrderSystem, scalar_t t, const state_vector_t& x, const input_vector_t& u,
                                              state_input_matrix_t& B) {
    // evaluation of f0 = f(x0,u0)
    state_vector_t f;
    system.computeFlowMap(t, x, u, f);

    for (size_t i = 0; i < INPUT_DIM; ++i) {
      // inspired from
      // http://en.wikipedia.org/wiki/Numerical_differentiation#Practical_considerations_using_floating_point_arithmetic
      scalar_t h = eps * std::max(fabs(u(i)), 1.0);

      input_vector_t uPlusPerturbed = u;
      uPlusPerturbed(i) += h;

      // get evaluation of f(x,u)
      state_vector_t fPlusPerturbed;
      system.computeFlowMap(t, x, uPlusPerturbed, fPlusPerturbed);

      if (doubleSidedDerivative) {
        input_vector_t uMinusPerturbed = u;
        uMinusPerturbed(i) -= h;

        state_vector_t fMinusPerturbed;
        system.computeFlowMap(t, x, uMinusPerturbed, fMinusPerturbed);

        if (isSecondOrderSystem) {
          B.template topRows<STATE_DIM / 2>().setZero();
          B.template block<STATE_DIM / 2, 1>(STATE_DIM / 2, i) =
              (fPlusPerturbed.template tail<STATE_DIM / 2>() - fMinusPerturbed.template tail<STATE_DIM / 2>()) / (2.0 * h);
        } else {
          B.col(i) = (fPlusPerturbed - fMinusPerturbed) / (2.0 * h);
        }
      } else {
        if (isSecondOrderSystem) {
          B.template topRows<STATE_DIM / 2>().setZero();
          B.template block<STATE_DIM / 2, 1>(STATE_DIM / 2, i) =
              (fPlusPerturbed.template tail<STATE_DIM / 2>() - f.template tail<STATE_DIM / 2>()) / h;
        } else {
          B.col(i) = (fPlusPerturbed - f) / h;
        }
      }
    }  // end of i loop
  }

  void finiteDifferenceDerivativeInput(scalar_t t, const state_vector_t& x, const input_vector_t& u, state_input_matrix_t& B) {
    return finiteDifferenceDerivativeInput(*controlledSystem_, eps_, doubleSidedDerivative_, isSecondOrderSystem_, t, x, u, B);
  }

  /**
   * Checks if the linearized dynamics using a finite difference method fall
   * within the tolerance.
   *
   * \note Each call will be at a different randomly perturbed state and control
   *
   * @param[in] controlledSystemBase
   * @param[in] derivativesBase
   * @param[in] t time to check at
   * @param[in] x state to check at
   * @param[in] u control to check at
   * @param[out] error  the numerical finite difference method to linearized
   * dynamic's error
   * @return whether the error is less than the tolerance specified in @property
   * tolerance_
   */
  static bool derivativeChecker(controlled_system_base_t& nonlinearSystem, derivatives_base_t& derivativesBase, scalar_t eps,
                                scalar_t tolerance, bool doubleSidedDerivative, bool isSecondOrderSystem, scalar_t t,
                                const state_vector_t& x, const input_vector_t& u, state_matrix_t& A_error, state_input_matrix_t& B_error) {
    state_matrix_t A;
    state_input_matrix_t B;
    derivativesBase.setCurrentStateAndControl(t, x, u);
    derivativesBase.getFlowMapDerivativeState(A);
    derivativesBase.getFlowMapDerivativeInput(B);

    finiteDifferenceDerivativeState(nonlinearSystem, eps, doubleSidedDerivative, isSecondOrderSystem, t, x, u, A_error);
    finiteDifferenceDerivativeInput(nonlinearSystem, eps, doubleSidedDerivative, isSecondOrderSystem, t, x, u, B_error);

    A_error -= A;
    B_error -= B;
    return tolerance > std::fmax(A_error.template lpNorm<Eigen::Infinity>(), B_error.template lpNorm<Eigen::Infinity>());
  }

  bool derivativeChecker(derivatives_base_t& derivativesBase, scalar_t t, const state_vector_t& x, const input_vector_t& u,
                         state_matrix_t& A_error, state_input_matrix_t& B_error) {
    return derivativeChecker(*controlledSystem_, derivativesBase, eps_, tolerance_, doubleSidedDerivative_, isSecondOrderSystem_, t, x, u,
                             A_error, B_error);
  }

 private:
  std::shared_ptr<controlled_system_base_t> controlledSystem_;
  scalar_t eps_;
  scalar_t tolerance_;
  bool doubleSidedDerivative_;
  bool isSecondOrderSystem_;
};

}  // namespace ocs2

#endif /* ifndef FINITEDIFFERENCEMETHODS_H */
