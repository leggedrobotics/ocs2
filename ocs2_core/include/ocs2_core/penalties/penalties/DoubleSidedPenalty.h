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

#include <memory>

#include <ocs2_core/penalties/penalties/PenaltyBase.h>

namespace ocs2 {

/**
 * Implements the double sided inequality \f$ l \leq h \leq u \f$ with a given penalty function \f$ p() \f$.
 *
 * \f[
 *   p_{box}(h) = p(h - l) + p(u - h)
 * \f]
 */
class DoubleSidedPenalty final : public PenaltyBase {
 public:
  /**
   * Constructor
   * @param [in] lowerBound: The lower bound.
   * @param [in] upperBound: The upper bound.
   * @param [in] penalty: The penalty for the two inequality constraint.
   */
  DoubleSidedPenalty(scalar_t lowerBound, scalar_t upperBound, std::unique_ptr<PenaltyBase> penalty)
      : lowerBound_(lowerBound), upperBound_(upperBound), penaltyPtr_(std::move(penalty)) {}

  ~DoubleSidedPenalty() override = default;
  DoubleSidedPenalty* clone() const override { return new DoubleSidedPenalty(*this); }
  std::string name() const override { return "DoubleSidedPenalty"; }

  scalar_t getValue(scalar_t t, scalar_t h) const override {
    return penaltyPtr_->getValue(t, h - lowerBound_) + penaltyPtr_->getValue(t, upperBound_ - h);
  }
  scalar_t getDerivative(scalar_t t, scalar_t h) const override {
    return penaltyPtr_->getDerivative(t, h - lowerBound_) - penaltyPtr_->getDerivative(t, upperBound_ - h);
  }
  scalar_t getSecondDerivative(scalar_t t, scalar_t h) const override {
    return penaltyPtr_->getSecondDerivative(t, h - lowerBound_) + penaltyPtr_->getSecondDerivative(t, upperBound_ - h);
  }

 private:
  DoubleSidedPenalty(const DoubleSidedPenalty& other)
      : lowerBound_(other.lowerBound_), upperBound_(other.upperBound_), penaltyPtr_(other.penaltyPtr_->clone()) {}

  const scalar_t lowerBound_;
  const scalar_t upperBound_;
  std::unique_ptr<PenaltyBase> penaltyPtr_;
};

}  // namespace ocs2
