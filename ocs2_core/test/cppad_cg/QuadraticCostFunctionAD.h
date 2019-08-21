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

#include "ocs2_core/cost/CostFunctionBaseAD.h"

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM>
class QuadraticCostFunctionAD : public CostFunctionBaseAD<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = CostFunctionBaseAD<STATE_DIM, INPUT_DIM>;
  using typename BASE::ad_dynamic_vector_t;
  using typename BASE::ad_scalar_t;
  using typename BASE::dynamic_vector_t;
  using typename BASE::input_matrix_t;
  using typename BASE::input_state_matrix_t;
  using typename BASE::input_vector_array_t;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_array_t;
  using typename BASE::scalar_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_array_t;
  using typename BASE::state_vector_t;

  QuadraticCostFunctionAD(const state_matrix_t& Q, const input_matrix_t& R, const state_vector_t& xNominal, const input_vector_t& uNominal,
                          const state_matrix_t& QFinal, const state_vector_t& xFinal,
                          const input_state_matrix_t& P = input_state_matrix_t::Zero())
      : BASE(), Q_(Q), R_(R), P_(P), QFinal_(QFinal), xNominal_(xNominal), uNominal_(uNominal), xFinal_(xFinal) {}

  QuadraticCostFunctionAD(const QuadraticCostFunctionAD& rhs)
      : BASE(rhs),
        Q_(rhs.Q_),
        R_(rhs.R_),
        P_(rhs.P_),
        QFinal_(rhs.QFinal_),
        xNominal_(rhs.xNominal_),
        uNominal_(rhs.uNominal_),
        xFinal_(rhs.xFinal_){};

  ~QuadraticCostFunctionAD() override = default;

  QuadraticCostFunctionAD* clone() const override { return new QuadraticCostFunctionAD(*this); }

 protected:
  dynamic_vector_t getIntermediateParameters(scalar_t time) const override {
    dynamic_vector_t parameters(STATE_DIM + INPUT_DIM);
    parameters << xNominal_, uNominal_;
    return parameters;
  }

  size_t getNumIntermediateParameters() const override { return STATE_DIM + INPUT_DIM; };

  dynamic_vector_t getTerminalParameters(scalar_t time) const override { return xFinal_; }

  size_t getNumTerminalParameters() const override { return STATE_DIM; };

  void intermediateCostFunction(ad_scalar_t time, const ad_dynamic_vector_t& state, const ad_dynamic_vector_t& input,
                                const ad_dynamic_vector_t& parameters, ad_scalar_t& costValue) const {
    ad_dynamic_vector_t stateDesired = parameters.template head<STATE_DIM>();
    ad_dynamic_vector_t inputDesired = parameters.template tail<INPUT_DIM>();
    ad_dynamic_vector_t xDeviation = state - stateDesired;
    ad_dynamic_vector_t uDeviation = input - inputDesired;

    costValue = 0.5 * xDeviation.dot(Q_.template cast<ad_scalar_t>() * xDeviation) +
                0.5 * uDeviation.dot(R_.template cast<ad_scalar_t>() * uDeviation) +
                uDeviation.dot(P_.template cast<ad_scalar_t>() * xDeviation);
  }

  void terminalCostFunction(ad_scalar_t time, const ad_dynamic_vector_t& state, const ad_dynamic_vector_t& parameters,
                            ad_scalar_t& costValue) const {
    ad_dynamic_vector_t stateDesired = parameters.template head<STATE_DIM>();
    ad_dynamic_vector_t xDeviation = state - stateDesired;
    costValue = 0.5 * xDeviation.dot(QFinal_.template cast<ad_scalar_t>() * xDeviation);
  }

 private:
  state_matrix_t Q_;
  input_matrix_t R_;
  input_state_matrix_t P_;
  state_matrix_t QFinal_;

  state_vector_t xNominal_;
  input_vector_t uNominal_;
  state_vector_t xFinal_;
};

}  // namespace ocs2
