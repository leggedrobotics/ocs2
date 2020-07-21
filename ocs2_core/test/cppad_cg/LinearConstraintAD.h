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

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/ConstraintBaseAD.h>

namespace ocs2 {

class LinearConstraintAD : public ConstraintBaseAD {
 public:
  using ConstraintBaseAD::ad_scalar_t;
  using ConstraintBaseAD::ad_vector_t;

  LinearConstraintAD(const vector_t& e, const matrix_t& C, const matrix_t& D, const vector_t& h, const matrix_t& F, const vector_t& h_f,
                     const matrix_t& F_f)
      : ConstraintBaseAD(C.cols(), D.cols()), e_(e), C_(C), D_(D), h_(h), F_(F), h_f_(h_f), F_f_(F_f) {}

  ~LinearConstraintAD() override = default;

  LinearConstraintAD(const LinearConstraintAD& rhs)
      : ConstraintBaseAD(rhs), e_(rhs.e_), C_(rhs.C_), D_(rhs.D_), h_(rhs.h_), F_(rhs.F_), h_f_(rhs.h_f_), F_f_(rhs.F_f_) {}

  LinearConstraintAD* clone() const override { return new LinearConstraintAD(*this); }

 protected:
  ad_vector_t stateInputConstraint(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input) const override {
    return e_.cast<ad_scalar_t>() + C_.cast<ad_scalar_t>() * state + D_.cast<ad_scalar_t>() * input;
  }

  ad_vector_t stateOnlyConstraint(ad_scalar_t time, const ad_vector_t& state) const override {
    return h_.cast<ad_scalar_t>() + F_.cast<ad_scalar_t>() * state;
  }

  ad_vector_t stateOnlyFinalConstraint(ad_scalar_t time, const ad_vector_t& state) const override {
    return h_f_.cast<ad_scalar_t>() + F_f_.cast<ad_scalar_t>() * state;
  }

 private:
  vector_t e_;
  matrix_t C_;
  matrix_t D_;
  vector_t h_;
  matrix_t F_;
  vector_t h_f_;
  matrix_t F_f_;
};

}  // namespace ocs2
