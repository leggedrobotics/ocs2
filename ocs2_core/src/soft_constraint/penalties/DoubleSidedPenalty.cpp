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

#include <ocs2_core/soft_constraint/penalties/DoubleSidedPenalty.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
DoubleSidedPenalty::DoubleSidedPenalty(const DoubleSidedPenalty& other)
    : lowerBound_(other.lowerBound_), upperBound_(other.upperBound_), penaltyPtr_(other.penaltyPtr_->clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t DoubleSidedPenalty::getValue(scalar_t t, scalar_t h) const {
  return penaltyPtr_->getValue(t, h - lowerBound_) + penaltyPtr_->getValue(t, upperBound_ - h);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t DoubleSidedPenalty::getDerivative(scalar_t t, scalar_t h) const {
  return penaltyPtr_->getDerivative(t, h - lowerBound_) - penaltyPtr_->getDerivative(t, upperBound_ - h);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t DoubleSidedPenalty::getSecondDerivative(scalar_t t, scalar_t h) const {
  return penaltyPtr_->getSecondDerivative(t, h - lowerBound_) + penaltyPtr_->getSecondDerivative(t, upperBound_ - h);
}

}  // namespace ocs2
