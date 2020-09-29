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

#include <ocs2_core/misc/LinearInterpolation.h>

#include "RiccatiModification.h"

// Declares an access function of name FIELD (e.g., time, DmDagger, ...)
#define CREATE_INTERPOLATION_ACCESS_FUNCTION(FIELD)                                                                             \
  inline auto FIELD(const std::vector<ocs2::riccati_modification::Data>& vec, size_t ind)->const decltype(vec[ind].FIELD##_)& { \
    return vec[ind].FIELD##_;                                                                                                   \
  }

namespace ocs2 {
namespace riccati_modification {

CREATE_INTERPOLATION_ACCESS_FUNCTION(time)

CREATE_INTERPOLATION_ACCESS_FUNCTION(deltaQm)
CREATE_INTERPOLATION_ACCESS_FUNCTION(deltaGm)
CREATE_INTERPOLATION_ACCESS_FUNCTION(deltaGv)

CREATE_INTERPOLATION_ACCESS_FUNCTION(hamiltonianHessian)
CREATE_INTERPOLATION_ACCESS_FUNCTION(constraintRangeProjector)
CREATE_INTERPOLATION_ACCESS_FUNCTION(constraintNullProjector)

}  // namespace riccati_modification
}  // namespace ocs2

#undef CREATE_INTERPOLATION_ACCESS_FUNCTION
