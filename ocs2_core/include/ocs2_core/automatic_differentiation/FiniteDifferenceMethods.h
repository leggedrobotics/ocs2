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
 * @tparam class DerivativesBase
 */
template <class ControlledSystemBase>
class FiniteDifferenceMethods {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef FiniteDifferenceMethods<ControlledSystemBase> finite_difference_method_t;
  typedef ControlledSystemBase controlled_system_base_t;
  typedef DerivativesBase<controlled_system_base_t::DIMENSIONS::DIMS::STATE_DIM_, controlled_system_base_t::DIMENSIONS::DIMS::INPUT_DIM_> derivatives_base_t;
  using scalar_t             = typename derivatives_base_t::DIMENSIONS::scalar_t;
  using state_vector_t       = typename derivatives_base_t::state_vector_t;
  using input_vector_t       = typename derivatives_base_t::input_vector_t;
  using state_matrix_t       = typename derivatives_base_t::state_matrix_t;
  using state_input_matrix_t = typename derivatives_base_t::state_input_matrix_t;

public:
  static inline void finiteDifferenceDerivativeState(controlled_system_base_t* csb_p, scalar_t eps, const state_vector_t& x, const input_vector_t& u, const scalar_t t, const state_vector_t& f_, state_matrix_t &A, bool doubleSidedDerivative, bool isSecondOrderSystem) {

    const auto STATE_DIM = ControlledSystemBase::DIMENSIONS::STATE_DIM_;
    const auto INPUT_DIM = ControlledSystemBase::DIMENSIONS::INPUT_DIM_;
    for (size_t i = 0; i < STATE_DIM; ++i) {

      // inspired from
      // http://en.wikipedia.org/wiki/Numerical_differentiation#Practical_considerations_using_floating_point_arithmetic
      scalar_t h = eps * std::max(fabs(x(i)), 1.0);

      state_vector_t xPlusPerturbed = x;
      xPlusPerturbed(i) += h;

      // get evaluation of f(x,u)
      state_vector_t fPlusPerturbed;
      csb_p->computeFlowMap(t, xPlusPerturbed, u, fPlusPerturbed);

      if (doubleSidedDerivative) {

        state_vector_t xMinusPerturbed = x;
        xMinusPerturbed(i) -= h;

        state_vector_t fMinusPerturbed;
        csb_p->computeFlowMap(t, xMinusPerturbed, u, fMinusPerturbed);

        if (isSecondOrderSystem) {
            std::cout << STATE_DIM;
          A.template topLeftCorner<STATE_DIM / 2, STATE_DIM / 2>().setZero();
          A.template topRightCorner<STATE_DIM / 2, STATE_DIM / 2>()
              .setIdentity();
          A.template block<STATE_DIM / 2, 1>(STATE_DIM / 2, i) =
              (fPlusPerturbed.template tail<STATE_DIM / 2>() -
               fMinusPerturbed.template tail<STATE_DIM / 2>()) /
              (2.0 * h);
        } else {
          A.col(i) = (fPlusPerturbed - fMinusPerturbed) / (2.0 * h);
        }
      } else {
        if (isSecondOrderSystem) {
          A.template topLeftCorner<STATE_DIM / 2, STATE_DIM / 2>().setZero();
          A.template topRightCorner<STATE_DIM / 2, STATE_DIM / 2>()
              .setIdentity();
          A.template block<STATE_DIM / 2, 1>(STATE_DIM / 2, i) =
              (fPlusPerturbed.template tail<STATE_DIM / 2>() -
               f_.template tail<STATE_DIM / 2>()) /
              h;
        } else {
          A.col(i) = (fPlusPerturbed - f_) / h;
        }
      }
    } // end of i loop
  }

  inline void finiteDifferenceDerivativeState(const state_vector_t& x,  const input_vector_t& u, const scalar_t t, state_matrix_t &A) {
    return finiteDifferenceDerivativeState(this->csb_p_.get(), this->eps_, x, u, t, this->f_,  A, this->doubleSidedDerivative_, this->isSecondOrderSystem_);
  }

  static inline void finiteDifferenceDerivativeInput(controlled_system_base_t* csb_p, const scalar_t eps, const input_vector_t& u, const state_vector_t& x, const scalar_t t, const state_vector_t& f_, state_input_matrix_t &Bout, bool doubleSidedDerivative, bool isSecondOrderSystem) {

    const auto STATE_DIM = ControlledSystemBase::DIMENSIONS::STATE_DIM_;
    const auto INPUT_DIM = ControlledSystemBase::DIMENSIONS::INPUT_DIM_;

    for (size_t i = 0; i < INPUT_DIM; ++i) {

      // inspired from
      // http://en.wikipedia.org/wiki/Numerical_differentiation#Practical_considerations_using_floating_point_arithmetic
      scalar_t h = eps * std::max(fabs(u(i)), 1.0);

      input_vector_t uPlusPerturbed = u;
      uPlusPerturbed(i) += h;

      // get evaluation of f(x,u)
      state_vector_t fPlusPerturbed;
      csb_p->computeFlowMap(t, x, uPlusPerturbed, fPlusPerturbed);

      if (doubleSidedDerivative) {

        input_vector_t uMinusPerturbed = u;
        uMinusPerturbed(i) -= h;

        state_vector_t fMinusPerturbed;
        csb_p->computeFlowMap(t, x, uMinusPerturbed,
                                            fMinusPerturbed);

        if (isSecondOrderSystem) {
          Bout.template topRows<STATE_DIM / 2>().setZero();
          Bout.template block<STATE_DIM / 2, 1>(STATE_DIM / 2, i) =
              (fPlusPerturbed.template tail<STATE_DIM / 2>() -
               fMinusPerturbed.template tail<STATE_DIM / 2>()) /
              (2.0 * h);
        } else {
          Bout.col(i) = (fPlusPerturbed - fMinusPerturbed) / (2.0 * h);
        }
      } else {
        if (isSecondOrderSystem) {
          Bout.template topRows<STATE_DIM / 2>().setZero();
          Bout.template block<STATE_DIM / 2, 1>(STATE_DIM / 2, i) =
              (fPlusPerturbed.template tail<STATE_DIM / 2>() -
               f_.template tail<STATE_DIM / 2>()) /
              h;
        } else {
          Bout.col(i) = (fPlusPerturbed - f_) / h;
        }
      }
    } // end of i loop
  }

  inline void finiteDifferenceDerivativeInput(const input_vector_t& u, const state_vector_t& x, const scalar_t t, state_input_matrix_t& Bout) {
    return finiteDifferenceDerivativeInput(this->csb_p_.get(), this->eps_, u, x, t, this->f_, Bout, this->doubleSidedDerivative_, this->isSecondOrderSystem_);
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

  static bool derivativeChecker(const std::shared_ptr<controlled_system_base_t>& nonlinearSystemPtr, 
      const std::shared_ptr<derivatives_base_t>& derivativesBasePtr,
      const scalar_t t, const state_vector_t &x, const input_vector_t &u,
      state_matrix_t &A_error, state_input_matrix_t &B_error,
      scalar_t eps, scalar_t tolerance,
      bool doubleSidedDerivative, bool isSecondOrderSystem)
  {
    state_vector_t flowmap;
    derivativesBasePtr->setCurrentStateAndControl(t, x, u);
    nonlinearSystemPtr->computeFlowMap(t, x, u, flowmap);

    state_matrix_t A;
    state_input_matrix_t B;
    derivativesBasePtr->getFlowMapDerivativeState(A);
    derivativesBasePtr->getFlowMapDerivativeInput(B);

    finite_difference_method_t::finiteDifferenceDerivativeState(nonlinearSystemPtr.get(), eps, x, u, t, flowmap, A_error, doubleSidedDerivative, isSecondOrderSystem);
    finite_difference_method_t::finiteDifferenceDerivativeInput(nonlinearSystemPtr.get(), eps, u, x, t, flowmap, B_error, doubleSidedDerivative, isSecondOrderSystem);

    A_error -= A;
    B_error -= B;
    // A_error.cwiseAbs();
    // B_error.cwiseAbs();
    return tolerance > std::fmax(A_error.template lpNorm<Eigen::Infinity>(),
                                 B_error.template lpNorm<Eigen::Infinity>());
  }

  bool derivativeChecker(
      const std::shared_ptr<derivatives_base_t>& derivativesBasePtr,
      const scalar_t t, const state_vector_t &x, const input_vector_t &u,
      state_matrix_t &A_error, state_input_matrix_t &B_error)
  {
    derivativesBasePtr->setCurrentStateAndControl(t, x, u);
    csb_p_->computeFlowMap(t, x, u, f_);

    state_matrix_t A;
    state_input_matrix_t B;
    derivativesBasePtr->getFlowMapDerivativeState(A);
    derivativesBasePtr->getFlowMapDerivativeInput(B);

    this->finiteDifferenceDerivativeState(x, u, t, A_error);
    this->finiteDifferenceDerivativeInput(u, x, t, B_error);

    A_error -= A;
    B_error -= B;
    // A_error.cwiseAbs();
    // B_error.cwiseAbs();
    return tolerance_ > std::fmax(A_error.template lpNorm<Eigen::Infinity>(),
                                 B_error.template lpNorm<Eigen::Infinity>());
  }

  /**********************************************************************************************
   *                                Constructors and Destructors *
   **********************************************************************************************/

  FiniteDifferenceMethods(
      const scalar_t eps         = Eigen::NumTraits<scalar_t>::epsilon(),
      const scalar_t tolerance   = sqrt(Eigen::NumTraits<scalar_t>::epsilon()),
      const std::shared_ptr<controlled_system_base_t>& csb_p = nullptr,
      bool doubleSidedDerivative = true, bool isSecondOrderSystem = false
      )
      : eps_(eps), tolerance_(tolerance), csb_p_(csb_p),
        doubleSidedDerivative_(doubleSidedDerivative),
        isSecondOrderSystem_(isSecondOrderSystem){};

  // FiniteDifferenceMethods(
  //     const scalar_t eps = Eigen::NumTraits<scalar_t>::epsilon(),
  //     const scalar_t tolerance = sqrt(Eigen::NumTraits<scalar_t>::epsilon()),
  //     SystemDynamicsLinearizer <ControlledSystemBase::DIMENSIONS::DIMS.STATE_DIM_, ControlledSystemBase::DIMENSIONS::DIMS.INPUT_DIM_>* sdl
  //     )
  //     : csb_p(sdl->csb_p), eps_(eps), tolerance_(tolerance), f_(sdl->f_),
  //       doubleSidedDerivative_(sdl->doubleSidedDerivative),
  //       isSecondOrderSystem_(sdl->isSecondOrderSystem)
  // {};

      /**
       * Copy constructor
       *
       * @param [in] other: Instance of the other class.
       */
      FiniteDifferenceMethods(const FiniteDifferenceMethods& other) :
        csb_p_(other.csb_p_->clone()), eps_(other.eps_), tolerance_(other.tolerance_), f_(other.f_),
        doubleSidedDerivative_(other.doubleSidedDerivative_),
        isSecondOrderSystem_(other.isSecondOrderSystem_)
  {};

      FiniteDifferenceMethods& operator=(const FiniteDifferenceMethods& other)  {
        eps_ = other.eps_;
        csb_p_ = other.csb_p_.clone();
        f_ = other.f_;
        doubleSidedDerivative_ = other.doubleSidedDerivative_;
        isSecondOrderSystem_ = other.isSecondOrderSystem_;
      }

  virtual ~FiniteDifferenceMethods(){};

private:
  const std::shared_ptr<controlled_system_base_t> csb_p_;
  scalar_t eps_;
  scalar_t tolerance_;
  state_vector_t f_;
  bool doubleSidedDerivative_;
  bool isSecondOrderSystem_;


}; // FiniteDifferenceMethods
//
} // namespace ocs2

#endif /* ifndef FINITEDIFFERENCEMETHODS_H */
