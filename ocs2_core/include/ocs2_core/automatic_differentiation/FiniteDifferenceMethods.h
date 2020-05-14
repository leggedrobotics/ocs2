/******************************************************************************
  Copyright (c) 2019, Oliver Harley. All rights reserved.

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
#include <ocs2_core/dynamics/ControlledSystemBase.h>

namespace ocs2 {

matrix_t finiteDifferenceDerivative(std::function<vector_t(const vector_t&)> f, const vector_t& x0,
                                    scalar_t eps = Eigen::NumTraits<scalar_t>::epsilon(), bool doubleSidedDerivative = true);

matrix_t finiteDifferenceDerivativeState(ControlledSystemBase& system, scalar_t t, const vector_t& x, const vector_t& u,
                                         scalar_t eps = Eigen::NumTraits<scalar_t>::epsilon(), bool doubleSidedDerivative = true,
                                         bool isSecondOrderSystem = false);

matrix_t finiteDifferenceDerivativeInput(ControlledSystemBase& system, scalar_t t, const vector_t& x, const vector_t& u,
                                         scalar_t eps = Eigen::NumTraits<scalar_t>::epsilon(), bool doubleSidedDerivative = true,
                                         bool isSecondOrderSystem = false);
}  // namespace ocs2
