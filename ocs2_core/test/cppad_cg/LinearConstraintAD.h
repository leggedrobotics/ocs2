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

#include "ocs2_core/constraint/ConstraintBaseAD.h"

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM>
class LinearConstraintAD : public ConstraintBaseAD<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = ConstraintBaseAD<STATE_DIM, INPUT_DIM>;
  using typename BASE::ad_dynamic_vector_t;
  using typename BASE::ad_scalar_t;
  using typename BASE::constraint1_input_matrix_t;
  using typename BASE::constraint1_state_matrix_t;
  using typename BASE::constraint1_vector_t;
  using typename BASE::constraint2_state_matrix_t;
  using typename BASE::constraint2_vector_t;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_t;
  using typename BASE::state_input_matrix_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_t;

  LinearConstraintAD(size_t numStateInputConstraint, const constraint1_vector_t& e, const constraint1_state_matrix_t& C,
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
        F_f_(F_f) {}

  ~LinearConstraintAD() override = default;

  LinearConstraintAD(const LinearConstraintAD& rhs)
      : BASE(rhs),
        numStateInputConstraint_(rhs.numStateInputConstraint_),
        e_(rhs.e_),
        C_(rhs.C_),
        D_(rhs.D_),
        numStateOnlyConstraint_(rhs.numStateOnlyConstraint_),
        h_(rhs.h_),
        F_(rhs.F_),
        numStateOnlyFinalConstraint_(rhs.numStateOnlyFinalConstraint_),
        h_f_(rhs.h_f_),
        F_f_(rhs.F_f_) {}

  LinearConstraintAD* clone() const override { return new LinearConstraintAD(*this); }

  size_t numStateInputConstraint(const scalar_t& time) override { return numStateInputConstraint_; }

  size_t numStateOnlyConstraint(const scalar_t& time) override { return numStateOnlyConstraint_; }

  size_t numStateOnlyFinalConstraint(const scalar_t& time) override { return numStateOnlyFinalConstraint_; }

 protected:
  void stateInputConstraint(ad_scalar_t time, const ad_dynamic_vector_t& state, const ad_dynamic_vector_t& input,
                            ad_dynamic_vector_t& constraintVector) const override {
    constraintVector = e_.template cast<ad_scalar_t>() + C_.template cast<ad_scalar_t>() * state + D_.template cast<ad_scalar_t>() * input;
  }

  void stateOnlyConstraint(ad_scalar_t time, const ad_dynamic_vector_t& state, ad_dynamic_vector_t& constraintVector) const override {
    constraintVector = h_.template cast<ad_scalar_t>() + F_.template cast<ad_scalar_t>() * state;
  }

  void stateOnlyFinalConstraint(ad_scalar_t time, const ad_dynamic_vector_t& state, ad_dynamic_vector_t& constraintVector) const override {
    constraintVector = h_f_.template cast<ad_scalar_t>() + F_f_.template cast<ad_scalar_t>() * state;
  }

 private:
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
};

}  // namespace ocs2
