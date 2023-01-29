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

#include "ocs2_oc/multiple_shooting/Initialization.h"

namespace ocs2 {
namespace multiple_shooting {

void initializeStateInputTrajectories(const vector_t& initState, const std::vector<AnnotatedTime>& timeDiscretization,
                                      const PrimalSolution& primalSolution, Initializer& initializer, vector_array_t& stateTrajectory,
                                      vector_array_t& inputTrajectory) {
  const int N = static_cast<int>(timeDiscretization.size()) - 1;  // // size of the input trajectory
  stateTrajectory.clear();
  stateTrajectory.reserve(N + 1);
  inputTrajectory.clear();
  inputTrajectory.reserve(N);

  // Determine till when to use the previous solution
  scalar_t interpolateStateTill = timeDiscretization.front().time;
  scalar_t interpolateInputTill = timeDiscretization.front().time;
  if (primalSolution.timeTrajectory_.size() >= 2) {
    interpolateStateTill = primalSolution.timeTrajectory_.back();
    interpolateInputTill = primalSolution.timeTrajectory_[primalSolution.timeTrajectory_.size() - 2];
  }

  // Initial state
  const scalar_t initTime = getIntervalStart(timeDiscretization[0]);
  if (initTime < interpolateStateTill) {
    stateTrajectory.push_back(LinearInterpolation::interpolate(initTime, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_));
  } else {
    stateTrajectory.push_back(initState);
  }

  for (int i = 0; i < N; i++) {
    if (timeDiscretization[i].event == AnnotatedTime::Event::PreEvent) {
      // Event Node
      inputTrajectory.push_back(vector_t());  // no input at event node
      stateTrajectory.push_back(initializeEventNode(timeDiscretization[i].time, stateTrajectory.back()));
    } else {
      // Intermediate node
      const scalar_t time = getIntervalStart(timeDiscretization[i]);
      const scalar_t nextTime = getIntervalEnd(timeDiscretization[i + 1]);
      vector_t input, nextState;
      if (time > interpolateInputTill || nextTime > interpolateStateTill) {  // Using initializer
        std::tie(input, nextState) = initializeIntermediateNode(initializer, time, nextTime, stateTrajectory.back());
      } else {  // interpolate previous solution
        std::tie(input, nextState) = initializeIntermediateNode(primalSolution, time, nextTime);
      }
      inputTrajectory.push_back(std::move(input));
      stateTrajectory.push_back(std::move(nextState));
    }
  }
}

}  // namespace multiple_shooting
}  // namespace ocs2
