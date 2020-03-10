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

//
// Created by rgrandia on 28.02.20.
//

#include "ocs2_qp_solver/QpTrajectories.h"

namespace ocs2 {
namespace qp_solver {

ContinuousTrajectory add(const ContinuousTrajectory& t0, const ContinuousTrajectory& t1) {
  const int N = t0.timeTrajectory.size() - 1;
  ContinuousTrajectory sum;
  sum.timeTrajectory = t0.timeTrajectory;
  sum.inputTrajectory.reserve(N);
  sum.stateTrajectory.reserve(N + 1);

  for (int k = 0; k < N; ++k) {
    sum.inputTrajectory.emplace_back(t0.inputTrajectory[k] + t1.inputTrajectory[k]);
    sum.stateTrajectory.emplace_back(t0.stateTrajectory[k] + t1.stateTrajectory[k]);
  }
  sum.stateTrajectory.emplace_back(t0.stateTrajectory[N] + t1.stateTrajectory[N]);
  return sum;
}

}  // namespace qp_solver
}  // namespace ocs2
