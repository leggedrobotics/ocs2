/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_mobile_manipulator_example/definitions.h>

namespace mobile_manipulator {

class QuadraticInputCost final : public ocs2::StateInputCost {
 public:
  explicit QuadraticInputCost(matrix_t R) : R_(std::move(R)) {}
  ~QuadraticInputCost() override = default;

  QuadraticInputCost* clone() const override { return new QuadraticInputCost(*this); }

  scalar_t getValue(scalar_t t, const vector_t& x, const vector_t& u,
                    const ocs2::CostDesiredTrajectories& desiredTrajectory) const override {
    const vector_t uDeviation = u - desiredTrajectory.getDesiredInput(t);
    return 0.5 * uDeviation.dot(R_ * uDeviation);
  }

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                 const ocs2::CostDesiredTrajectories& desiredTrajectory) const override {
    const vector_t uDeviation = u - desiredTrajectory.getDesiredInput(t);
    const vector_t rDeviation = R_ * uDeviation;

    ScalarFunctionQuadraticApproximation L = ScalarFunctionQuadraticApproximation::Zero(x.rows(), u.rows());
    L.f = 0.5 * uDeviation.dot(rDeviation);
    L.dfdu = rDeviation;
    L.dfduu = R_;
    return L;
  }

 private:
  matrix_t R_;
};

}  // namespace mobile_manipulator
