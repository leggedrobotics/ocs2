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

#pragma once

#include <ostream>

#include "ocs2_core/Types.h"

namespace ocs2 {

/**
 * This class is an interface class for the user defined target trajectories.
 */
struct TargetTrajectories {
  explicit TargetTrajectories(size_t size = 0);
  TargetTrajectories(scalar_array_t desiredTimeTrajectory, vector_array_t desiredStateTrajectory,
                     vector_array_t desiredInputTrajectory = vector_array_t());
  void clear();
  bool empty() const { return timeTrajectory.empty() || stateTrajectory.empty(); }
  size_t size() const { return timeTrajectory.size(); }

  bool operator==(const TargetTrajectories& other);
  bool operator!=(const TargetTrajectories& other) { return !(*this == other); }

  vector_t getDesiredState(scalar_t time) const;
  vector_t getDesiredInput(scalar_t time) const;

  scalar_array_t timeTrajectory;
  vector_array_t stateTrajectory;
  vector_array_t inputTrajectory;
};

void swap(TargetTrajectories& lh, TargetTrajectories& rh);
std::ostream& operator<<(std::ostream& out, const TargetTrajectories& targetTrajectories);

}  // namespace ocs2
