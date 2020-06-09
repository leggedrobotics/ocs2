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

#include <ocs2_core/Types.h>

namespace ocs2 {

class TransferFunctionBase {
 public:
  TransferFunctionBase(vector_t numCoefficients, vector_t denCoefficients, scalar_t timedelay = 0.0, bool balance = true);

  void absorbDelay(size_t numZeros, size_t numPoles);

  void normalize();

  void getStateSpace(matrix_t& A, matrix_t& B, matrix_t& C, matrix_t& D);

 private:
  vector_t numCoefficients_;
  vector_t denCoefficients_;
  scalar_t timeDelay_;
  const scalar_t delayTol_ = 1e-6;
  bool delayAbsorbed_ = false;
  bool balance_;
};

inline void tf2ss(vector_t numCoefficients, vector_t denCoefficients, matrix_t& A, matrix_t& B, matrix_t& C, matrix_t& D,
                  scalar_t timeDelay = 0.0, bool balance = true) {
  TransferFunctionBase tf(numCoefficients, denCoefficients, timeDelay, balance);
  tf.getStateSpace(A, B, C, D);
}

};  // namespace ocs2
