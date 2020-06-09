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

#include <utility>

#include <ocs2_core/control/ControllerBase.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ControllerBase::ControllerBase(size_t stateDim, size_t inputDim) : stateDim_(stateDim), inputDim_(inputDim) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ControllerBase::concatenate(const ControllerBase* otherController) {
  concatenate(otherController, 0, otherController->size());
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
void ControllerBase::flatten(const scalar_array_t& timeArray, const std::vector<std::vector<float>*>& flatArray2) const {
  throw std::runtime_error("not implemented.");
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
void ControllerBase::unFlatten(const scalar_array_t& timeArray, const std::vector<std::vector<float> const*>& flatArray2) {
  throw std::runtime_error("not implemented.");
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
void swap(ControllerBase& a, ControllerBase& b) noexcept {
  std::swap(a.stateDim_, b.stateDim_);
  std::swap(a.inputDim_, b.inputDim_);
}

}  // namespace ocs2
