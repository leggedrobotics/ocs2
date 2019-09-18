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

#include "ocs2_core/constraint/ConstraintBase.h"

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM>
class LinearConstraint final : public ConstraintBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<LinearConstraint<STATE_DIM, INPUT_DIM> >;
  using ConstPtr = std::shared_ptr<const LinearConstraint<STATE_DIM, INPUT_DIM> >;

  using BASE = ConstraintBase<STATE_DIM, INPUT_DIM>;
  using typename BASE::constraint1_input_matrix_t;
  using typename BASE::constraint1_state_matrix_t;
  using typename BASE::constraint1_vector_t;
  using typename BASE::constraint2_state_matrix_t;
  using typename BASE::constraint2_vector_t;
  using typename BASE::input_matrix_array_t;
  using typename BASE::input_state_matrix_array_t;
  using typename BASE::input_vector_array_t;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_array_t;
  using typename BASE::scalar_t;
  using typename BASE::state_input_matrix_t;
  using typename BASE::state_matrix_array_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_array_t;
  using typename BASE::state_vector_t;

  /**
   * @brief Constructor that sets the number constraints for all types to 0
   */
  LinearConstraint()
      : numStateInputConstraint_(0),
        e_(constraint1_vector_t::Zero()),
        C_(constraint1_state_matrix_t::Zero()),
        D_(constraint1_input_matrix_t::Zero()),
        numStateOnlyConstraint_(0),
        h_(constraint2_vector_t::Zero()),
        F_(constraint2_state_matrix_t::Zero()),
        numStateOnlyFinalConstraint_(0),
        h_f_(constraint2_vector_t::Zero()),
        F_f_(constraint2_state_matrix_t::Zero()),
        numInequalityConstraint_(0),
        h0_(),
        dhdx_(),
        dhdu_(),
        ddhdxdx_(),
        ddhdudu_(),
        ddhdudx_() {}

  /**
   * @brief Constructor for only equality constraints
   *
   * @param[in] numStateInputConstraint: Number of state-input equality constraints
   * @param[in] e: Constant term in C * x + D * u + e = 0
   * @param[in] C: x factor in C * x + D * u + e = 0
   * @param[in] D: u factor in C * x + D * u + e = 0
   * @param[in] numStateOnlyConstraint: Number of state-only equality constraints
   * @param[in] h: Constant term in F * x + h = 0
   * @param[in] F: x factor in F * x + h = 0
   * @param[in] numStateOnlyFinalConstraint: Number of final time state-only equality constrains
   * @param[in] h_f: Constant term in F_f * x + h_f = 0 (at final time)
   * @param[in] F_f: x factor in F_f * x + h_f = 0 (at final time)
   */
  LinearConstraint(size_t numStateInputConstraint, const constraint1_vector_t& e, const constraint1_state_matrix_t& C,
                   const constraint1_input_matrix_t& D, size_t numStateOnlyConstraint, const constraint2_vector_t& h,
                   const constraint2_state_matrix_t& F, size_t numStateOnlyFinalConstraint, const constraint2_vector_t& h_f,
                   const constraint2_state_matrix_t& F_f)
      : numStateInputConstraint_(numStateInputConstraint),
        e_(e),
        C_(C),
        D_(D),
        numStateOnlyConstraint_(numStateOnlyConstraint),
        h_(h),
        F_(F),
        numStateOnlyFinalConstraint_(numStateOnlyFinalConstraint),
        h_f_(h_f),
        F_f_(F_f),
        numInequalityConstraint_(0) {}

  /**
   * @brief General constructor for equality and inequality constraints
   * @note The inequality constraint can be quadratic of the form
   * h0 + x^T * dhdx + u^T * dhdu + x^T * ddhdxdx * x + u^T * ddhdudu * u + u^T * ddhdudx * x >= 0
   *
   * @param[in] numStateInputConstraint: Number of state-input equality constraints
   * @param[in] e: Constant term in C * x + D * u + e = 0
   * @param[in] C: x factor in C * x + D * u + e = 0
   * @param[in] D: u factor in C * x + D * u + e = 0
   * @param[in] numStateOnlyConstraint: Number of state-only equality constraints
   * @param[in] h: Constant term in F * x + h = 0
   * @param[in] F: x factor in F * x + h = 0
   * @param[in] numStateOnlyFinalConstraint: Number of final time state-only equality constrains
   * @param[in] h_f: Constant term in F_f * x + h_f = 0 (at final time)
   * @param[in] F_f: x factor in F_f * x + h_f = 0 (at final time)
   * @param[in] numInequalityConstraint: Number of inequality constraints
   * @param[in] h0: Constant term in inequality constraint
   * @param[in] dhdx: Linear x multiplier in inequality constraint
   * @param[in] dhdu: Linear u multiplier in inequality constraint
   * @param[in] ddhdxdx: Quadratic x multiplier in inequality constraint
   * @param[in] ddhdudu: Quadratic u multiplier in inequality constraint
   * @param[in] ddhdudx: Quadratic mixed term in inequality constraint
   */
  LinearConstraint(size_t numStateInputConstraint, const constraint1_vector_t& e, const constraint1_state_matrix_t& C,
                   const constraint1_input_matrix_t& D, size_t numStateOnlyConstraint, const constraint2_vector_t& h,
                   const constraint2_state_matrix_t& F, size_t numStateOnlyFinalConstraint, const constraint2_vector_t& h_f,
                   const constraint2_state_matrix_t& F_f, size_t numInequalityConstraint, const scalar_array_t& h0,
                   const state_vector_array_t& dhdx, const input_vector_array_t& dhdu, const state_matrix_array_t& ddhdxdx,
                   const input_matrix_array_t& ddhdudu, const input_state_matrix_array_t& ddhdudx)
      : numStateInputConstraint_(numStateInputConstraint),
        e_(e),
        C_(C),
        D_(D),
        numStateOnlyConstraint_(numStateOnlyConstraint),
        h_(h),
        F_(F),
        numStateOnlyFinalConstraint_(numStateOnlyFinalConstraint),
        h_f_(h_f),
        F_f_(F_f),
        numInequalityConstraint_(numInequalityConstraint),
        h0_(h0),
        dhdx_(dhdx),
        dhdu_(dhdu),
        ddhdxdx_(ddhdxdx),
        ddhdudu_(ddhdudu),
        ddhdudx_(ddhdudx) {}

  virtual ~LinearConstraint() = default;

  LinearConstraint<STATE_DIM, INPUT_DIM>* clone() const override { return new LinearConstraint<STATE_DIM, INPUT_DIM>(*this); }

  void getConstraint1(constraint1_vector_t& g1) override { g1 = e_ + C_ * BASE::x_ + D_ * BASE::u_; }

  size_t numStateInputConstraint(const scalar_t& time) override { return numStateInputConstraint_; }

  void getConstraint2(constraint2_vector_t& g2) override { g2 = h_ + F_ * BASE::x_; }

  size_t numStateOnlyConstraint(const scalar_t& time) override { return numStateOnlyConstraint_; }

  void getInequalityConstraint(scalar_array_t& h) override {
    h.clear();
    for (size_t i = 0; i < numInequalityConstraint_; i++) {
      h.emplace_back(h0_[i] + dhdx_[i].dot(BASE::x_) + dhdu_[i].dot(BASE::u_) + 0.5 * BASE::x_.dot(ddhdxdx_[i] * BASE::x_) +
                     0.5 * BASE::u_.dot(ddhdudu_[i] * BASE::u_) + BASE::u_.dot(ddhdudx_[i] * BASE::x_));
    }
  }

  size_t numInequalityConstraint(const scalar_t& time) override { return numInequalityConstraint_; };

  void getFinalConstraint2(constraint2_vector_t& g2Final) override { g2Final = h_f_ + F_f_ * BASE::x_; }

  size_t numStateOnlyFinalConstraint(const scalar_t& time) override { return numStateOnlyFinalConstraint_; }

  void getConstraint1DerivativesState(constraint1_state_matrix_t& C) override { C = C_; }
  void getConstraint1DerivativesControl(constraint1_input_matrix_t& D) override { D = D_; }

  void getConstraint2DerivativesState(constraint2_state_matrix_t& F) override { F = F_; }

  void getInequalityConstraintDerivativesState(state_vector_array_t& dhdx) override {
    dhdx.clear();
    for (size_t i = 0; i < numInequalityConstraint_; i++) {
      dhdx.push_back(dhdx_[i] + ddhdxdx_[i] * BASE::x_ + ddhdudx_[i].transpose() * BASE::u_);
    }
  }

  void getInequalityConstraintDerivativesInput(input_vector_array_t& dhdu) override {
    dhdu.clear();
    for (size_t i = 0; i < numInequalityConstraint_; i++) {
      dhdu.push_back(dhdu_[i] + ddhdudu_[i] * BASE::u_ + ddhdudx_[i] * BASE::x_);
    }
  }

  void getInequalityConstraintSecondDerivativesState(state_matrix_array_t& ddhdxdx) override { ddhdxdx = ddhdxdx_; }
  void getInequalityConstraintSecondDerivativesInput(input_matrix_array_t& ddhdudu) override { ddhdudu = ddhdudu_; }
  void getInequalityConstraintDerivativesInputState(input_state_matrix_array_t& ddhdudx) override { ddhdudx = ddhdudx_; }

  void getFinalConstraint2DerivativesState(constraint2_state_matrix_t& F_f) override { F_f = F_f_; }

 public:
  size_t numStateInputConstraint_;
  constraint1_vector_t e_;
  constraint1_state_matrix_t C_;
  constraint1_input_matrix_t D_;

  size_t numStateOnlyConstraint_;
  constraint2_vector_t h_;
  constraint2_state_matrix_t F_;

  size_t numStateOnlyFinalConstraint_;
  constraint2_vector_t h_f_;
  constraint2_state_matrix_t F_f_;

  size_t numInequalityConstraint_;
  scalar_array_t h0_;
  state_vector_array_t dhdx_;
  input_vector_array_t dhdu_;
  state_matrix_array_t ddhdxdx_;
  input_matrix_array_t ddhdudu_;
  input_state_matrix_array_t ddhdudx_;
};

}  // namespace ocs2
