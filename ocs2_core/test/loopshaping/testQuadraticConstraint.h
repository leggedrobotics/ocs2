/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_core/constraint/StateInputConstraint.h>

namespace ocs2 {

class TestQuadraticStateInputConstraint : public StateInputConstraint {
 public:
  TestQuadraticStateInputConstraint(matrix_t Q, matrix_t R, matrix_t P)
      : StateInputConstraint(ConstraintOrder::Quadratic), Q_(std::move(Q)), R_(std::move(R)), P_(std::move(P)) {}

  static std::unique_ptr<TestQuadraticStateInputConstraint> createRandom(size_t stateDim, size_t inputDim) {
    matrix_t Q, R, P;
    Q.setRandom(stateDim, stateDim);
    R.setRandom(inputDim, inputDim);
    P.setRandom(inputDim, stateDim);
    Q = (0.5 * Q.transpose() + 0.5 * Q).eval();
    R = (0.5 * R.transpose() + 0.5 * R).eval();
    return std::make_unique<TestQuadraticStateInputConstraint>(std::move(Q), std::move(R), std::move(P));
  }

  ~TestQuadraticStateInputConstraint() override = default;

  TestQuadraticStateInputConstraint* clone() const override { return new TestQuadraticStateInputConstraint(*this); }

  size_t getNumConstraints(scalar_t time) const final { return 1; }

  vector_t getValue(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) const final {
    vector_t c(1);
    c(0) = 0.5 * x.dot(Q_ * x) + 0.5 * u.dot(R_ * u) + u.dot(P_ * x);
    return c;
  }

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                           const PreComputation& preComp) const final {
    VectorFunctionLinearApproximation c;
    c.f = getValue(t, x, u, preComp);
    c.dfdx = x.transpose() * Q_ + u.transpose() * P_;
    c.dfdu = u.transpose() * R_ + x.transpose() * P_.transpose();
    return c;
  }

  VectorFunctionQuadraticApproximation getQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                 const PreComputation& preComp) const final {
    VectorFunctionQuadraticApproximation c;
    c.f = getValue(t, x, u, preComp);
    c.dfdx = x.transpose() * Q_ + u.transpose() * P_;
    c.dfdu = u.transpose() * R_ + x.transpose() * P_.transpose();
    c.dfdxx.push_back(Q_);
    c.dfduu.push_back(R_);
    c.dfdux.push_back(P_);
    return c;
  }

 public:
  matrix_t Q_;
  matrix_t R_;
  matrix_t P_;
};

class TestQuadraticStateConstraint : public StateConstraint {
 public:
  TestQuadraticStateConstraint(matrix_t Q) : StateConstraint(ConstraintOrder::Quadratic), Q_(std::move(Q)) {}

  static std::unique_ptr<TestQuadraticStateConstraint> createRandom(size_t stateDim) {
    matrix_t Q;
    Q.setRandom(stateDim, stateDim);
    Q = (0.5 * Q.transpose() + 0.5 * Q).eval();
    return std::make_unique<TestQuadraticStateConstraint>(std::move(Q));
  }

  ~TestQuadraticStateConstraint() override = default;

  TestQuadraticStateConstraint* clone() const override { return new TestQuadraticStateConstraint(*this); }

  size_t getNumConstraints(scalar_t time) const final { return 1; }

  vector_t getValue(scalar_t t, const vector_t& x, const PreComputation&) const final {
    vector_t c(1);
    c(0) = 0.5 * x.dot(Q_ * x);
    return c;
  }

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t t, const vector_t& x, const PreComputation& preComp) const final {
    VectorFunctionLinearApproximation c;
    c.f = getValue(t, x, preComp);
    c.dfdx = x.transpose() * Q_;
    return c;
  }

  VectorFunctionQuadraticApproximation getQuadraticApproximation(scalar_t t, const vector_t& x, const PreComputation& preComp) const final {
    VectorFunctionQuadraticApproximation c;
    c.f = getValue(t, x, preComp);
    c.dfdx = x.transpose() * Q_;
    c.dfdxx.push_back(Q_);
    return c;
  }

 public:
  matrix_t Q_;
};

}  // namespace ocs2
