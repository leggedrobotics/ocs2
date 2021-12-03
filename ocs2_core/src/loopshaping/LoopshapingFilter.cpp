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

#include "ocs2_core/loopshaping/LoopshapingFilter.h"

#include <iostream>

namespace ocs2 {

Filter::Filter() {
  A_.resize(0, 0);
  B_.resize(0, 0);
  C_.resize(0, 0);
  D_.resize(0, 0);
}

Filter::Filter(matrix_t A, matrix_t B, matrix_t C, matrix_t D)
    : A_(std::move(A)),
      B_(std::move(B)),
      C_(std::move(C)),
      D_(std::move(D)),
      a_(A_.diagonal()),
      b_(B_.diagonal()),
      c_(C_.diagonal()),
      d_(D_.diagonal()),
      numStates_(A_.rows()),
      numInputs_(B_.cols()),
      numOutputs_(C_.rows()) {
  checkSize();

  // precompute row + column scaling
  diagCC_ = c_ * matrix_t::Ones(c_.cols(), c_.rows()) * c_;
  diagDC_ = d_ * matrix_t::Ones(d_.cols(), c_.rows()) * c_;
  diagDD_ = d_ * matrix_t::Ones(d_.cols(), d_.rows()) * d_;

  // Prepare inv(A)
  if (A_.size() > 0) {
    Aqr_.compute(A_);
  }

  // Prepare inv(D)
  Dqr_.compute(D_);

  // Prepare inv([A, B; C, D])
  matrix_t ABCD(numStates_ + numInputs_, numStates_ + numInputs_);
  ABCD << A_, B_, C_, D_;
  ABCDqr_.compute(ABCD);
}

void Filter::print() const {
  std::cerr << "numStates: " << numStates_ << std::endl;
  std::cerr << "numInputs: " << numInputs_ << std::endl;
  std::cerr << "numOutputs: " << numOutputs_ << std::endl;
  auto printMatrix = [](const std::string& name, const matrix_t& m) {
    if (m.isDiagonal()) {
      std::cerr << name << ": (diagonal) \n" << m.diagonal().transpose() << "\n";
    } else {
      std::cerr << name << ": \n" << m << "\n";
    }
  };
  printMatrix("A", A_);
  printMatrix("B", B_);
  printMatrix("C", C_);
  printMatrix("D", D_);
}

void Filter::findEquilibriumForOutput(const vector_t& y, vector_t& x, vector_t& u) const {
  // Solve (given y)
  // [0  =  [  A    B    [x
  //  y]       C    D  ]  u]
  vector_t zero_y(numStates_ + numOutputs_);
  zero_y << vector_t::Zero(numStates_), y;

  const vector_t x_u = ABCDqr_.solve(zero_y);

  x = x_u.head(numStates_);
  u = x_u.tail(numInputs_);
}

void Filter::findEquilibriumForOutputGivenState(const vector_t& y, const vector_t& x, vector_t& u) const {
  // Solve (given y, x)
  //  y =  [  C    D  ] [x; u]
  vector_t tmp = y;
  tmp.noalias() -= C_ * x;
  u = Dqr_.solve(tmp);
}

void Filter::findEquilibriumForInput(const vector_t& u, vector_t& x, vector_t& y) const {
  // Solve (given u)
  // [0  =  [  A    B    [x
  //  y]       C    D  ]  u]
  y.noalias() = D_ * u;
  if (numStates_ > 0) {
    x = -Aqr_.solve(B_ * u);
    y.noalias() += C_ * x;
  }
}

void Filter::checkSize() const {
  bool correct = true;
  // check number of state
  correct &= numStates_ == A_.rows();
  correct &= numStates_ == B_.rows();
  correct &= numStates_ == C_.cols();
  // Check number of inputs
  correct &= numInputs_ == B_.cols();
  correct &= numInputs_ == D_.cols();
  // Check number of outputs
  correct &= numOutputs_ == C_.rows();
  correct &= numOutputs_ == D_.rows();
  if (!correct) {
    print();
    throw std::runtime_error("Loopshaping: Filer: Matrix dimensions not consistent.");
  }
}

}  // namespace ocs2
