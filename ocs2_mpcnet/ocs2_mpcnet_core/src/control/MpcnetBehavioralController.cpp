/******************************************************************************
Copyright (c) 2022, Farbod Farshidian. All rights reserved.

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

#include "ocs2_mpcnet_core/control/MpcnetBehavioralController.h"

#include <ocs2_core/misc/Numerics.h>

namespace ocs2 {
namespace mpcnet {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t MpcnetBehavioralController::computeInput(scalar_t t, const vector_t& x) {
  if (optimalControllerPtr_ != nullptr && learnedControllerPtr_ != nullptr) {
    if (numerics::almost_eq(alpha_, 0.0)) {
      return learnedControllerPtr_->computeInput(t, x);
    } else if (numerics::almost_eq(alpha_, 1.0)) {
      return optimalControllerPtr_->computeInput(t, x);
    } else {
      return alpha_ * optimalControllerPtr_->computeInput(t, x) + (1 - alpha_) * learnedControllerPtr_->computeInput(t, x);
    }
  } else {
    throw std::runtime_error(
        "[MpcnetBehavioralController::computeInput] cannot compute input, since optimal and/or learned controller not set.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
int MpcnetBehavioralController::size() const {
  if (optimalControllerPtr_ != nullptr && learnedControllerPtr_ != nullptr) {
    return std::max(optimalControllerPtr_->size(), learnedControllerPtr_->size());
  } else if (optimalControllerPtr_ != nullptr) {
    return optimalControllerPtr_->size();
  } else if (learnedControllerPtr_ != nullptr) {
    return learnedControllerPtr_->size();
  } else {
    return 0;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MpcnetBehavioralController::clear() {
  if (optimalControllerPtr_ != nullptr) {
    optimalControllerPtr_->clear();
  }
  if (learnedControllerPtr_ != nullptr) {
    learnedControllerPtr_->clear();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MpcnetBehavioralController::empty() const {
  if (optimalControllerPtr_ != nullptr && learnedControllerPtr_ != nullptr) {
    return optimalControllerPtr_->empty() && learnedControllerPtr_->empty();
  } else if (optimalControllerPtr_ != nullptr) {
    return optimalControllerPtr_->empty();
  } else if (learnedControllerPtr_ != nullptr) {
    return learnedControllerPtr_->empty();
  } else {
    return true;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MpcnetBehavioralController::concatenate(const ControllerBase* otherController, int index, int length) {
  if (optimalControllerPtr_ != nullptr) {
    optimalControllerPtr_->concatenate(otherController, index, length);
  }
  if (learnedControllerPtr_ != nullptr) {
    learnedControllerPtr_->concatenate(otherController, index, length);
  }
}

}  // namespace mpcnet
}  // namespace ocs2
