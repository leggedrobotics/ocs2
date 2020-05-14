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
 * This class is an interface class for the cost desired trajectories.
 */
class CostDesiredTrajectories {
 public:
  CostDesiredTrajectories(const scalar_array_t& desiredTimeTrajectory, const vector_array_t& desiredStateTrajectory,
                          const vector_array_t& desiredInputTrajectory);

  explicit CostDesiredTrajectories(size_t trajectorySize);

  ~CostDesiredTrajectories() = default;

  bool empty() const;

  void clear();

  void swap(CostDesiredTrajectories& other);

  bool operator==(const CostDesiredTrajectories& other);

  inline bool operator!=(const CostDesiredTrajectories& other) { return !(*this == other); }

  inline scalar_array_t& desiredTimeTrajectory() { return desiredTimeTrajectory_; }
  inline const scalar_array_t& desiredTimeTrajectory() const { return desiredTimeTrajectory_; }

  inline vector_array_t& desiredStateTrajectory() { return desiredStateTrajectory_; }
  inline const vector_array_t& desiredStateTrajectory() const { return desiredStateTrajectory_; }

  inline vector_array_t& desiredInputTrajectory() { return desiredInputTrajectory_; }
  inline const vector_array_t& desiredInputTrajectory() const { return desiredInputTrajectory_; }

  void getDesiredState(scalar_t time, vector_t& desiredState) const;
  void getDesiredInput(scalar_t time, vector_t& desiredInput) const;
  void display() const;

 private:
  scalar_array_t desiredTimeTrajectory_;
  vector_array_t desiredStateTrajectory_;
  vector_array_t desiredInputTrajectory_;

  friend std::ostream& operator<<(std::ostream& out, const CostDesiredTrajectories& traj);
};

std::ostream& operator<<(std::ostream& out, const CostDesiredTrajectories& traj);

}  // namespace ocs2
