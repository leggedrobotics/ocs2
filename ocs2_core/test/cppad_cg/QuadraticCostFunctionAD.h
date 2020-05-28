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
#include <ocs2_core/cost/CostFunctionBaseAD.h>

namespace ocs2 {

class QuadraticCostFunctionAD : public CostFunctionBaseAD {
 public:
  using typename CostFunctionBaseAD::ad_scalar_t;
  using typename CostFunctionBaseAD::ad_vector_t;

  QuadraticCostFunctionAD(const matrix_t& Q, const matrix_t& R, const vector_t& xNominal, const vector_t& uNominal, const matrix_t& QFinal,
                          const vector_t& xFinal, const matrix_t& P = matrix_t())
      : CostFunctionBaseAD(Q.rows(), R.rows()),
        Q_(Q),
        R_(R),
        P_(P),
        QFinal_(QFinal),
        xNominal_(xNominal),
        uNominal_(uNominal),
        xFinal_(xFinal) {
    if (P_.size() == 0) {
      P_ = matrix_t::Zero(R.rows(), Q.rows());
    }
  }

  QuadraticCostFunctionAD(const QuadraticCostFunctionAD& rhs)
      : CostFunctionBaseAD(rhs),
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
  vector_t getIntermediateParameters(scalar_t time) const override {
    vector_t parameters(stateDim_ + inputDim_);
    parameters << xNominal_, uNominal_;
    return parameters;
  }

  size_t getNumIntermediateParameters() const override { return stateDim_ + inputDim_; };

  vector_t getTerminalParameters(scalar_t time) const override { return xFinal_; }

  size_t getNumTerminalParameters() const override { return stateDim_; };

  ad_scalar_t intermediateCostFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                       const ad_vector_t& parameters) const override {
    ad_vector_t stateDesired = parameters.head(stateDim_);
    ad_vector_t inputDesired = parameters.tail(inputDim_);
    ad_vector_t xDeviation = state - stateDesired;
    ad_vector_t uDeviation = input - inputDesired;

    return 0.5 * xDeviation.dot(Q_.template cast<ad_scalar_t>() * xDeviation) +
           0.5 * uDeviation.dot(R_.template cast<ad_scalar_t>() * uDeviation) + uDeviation.dot(P_.cast<ad_scalar_t>() * xDeviation);
  }

  ad_scalar_t terminalCostFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const override {
    ad_vector_t stateDesired = parameters.head(stateDim_);
    ad_vector_t xDeviation = state - stateDesired;
    return 0.5 * xDeviation.dot(QFinal_.cast<ad_scalar_t>() * xDeviation);
  }

 private:
  matrix_t Q_;
  matrix_t R_;
  matrix_t P_;
  matrix_t QFinal_;

  vector_t xNominal_;
  vector_t uNominal_;
  vector_t xFinal_;
};

}  // namespace ocs2
