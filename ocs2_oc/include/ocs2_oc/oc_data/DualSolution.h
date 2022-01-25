#include <ocs2_oc/oc_data/PrimalSolution.h>/******************************************************************************
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

#include <ocs2_core/Types.h>
#include <ocs2_core/model_data/Multiplier.h>

namespace ocs2 {

struct DualSolution {
  scalar_array_t timeTrajectory;

  MultiplierCollection final;
  std::vector<MultiplierCollection> preJumps;
  std::vector<MultiplierCollection> intermediates;
};

struct DualSolutionRef {
  DualSolutionRef(DualSolution& dualSolution) : DualSolutionRef(dualSolution.final, dualSolution.preJumps, dualSolution.intermediates) {}

  DualSolutionRef(MultiplierCollection& finalRef, std::vector<MultiplierCollection>& preJumpsRef,
                  std::vector<MultiplierCollection>& intermediatesRef)
      : final(finalRef), preJumps(preJumpsRef), intermediates(intermediatesRef) {}

  MultiplierCollection& final;
  std::vector<MultiplierCollection>& preJumps;
  std::vector<MultiplierCollection>& intermediates;
};

struct DualSolutionConstRef {
  DualSolutionConstRef(const DualSolution& dualSolution)
      : DualSolutionConstRef(dualSolution.final, dualSolution.preJumps, dualSolution.intermediates) {}

  DualSolutionConstRef(const MultiplierCollection& finalRef, const std::vector<MultiplierCollection>& preJumpsRef,
                       const std::vector<MultiplierCollection>& intermediatesRef)
      : final(finalRef), preJumps(preJumpsRef), intermediates(intermediatesRef) {}

  const MultiplierCollection& final;
  const std::vector<MultiplierCollection>& preJumps;
  const std::vector<MultiplierCollection>& intermediates;
};

inline void swap(DualSolution& lhs, DualSolution& rhs) {
  swap(lhs.final, rhs.final);
  lhs.preJumps.swap(rhs.preJumps);
  lhs.intermediates.swap(rhs.intermediates);
  lhs.timeTrajectory.swap(rhs.timeTrajectory);
}

inline void clear(DualSolution& d) {
  clear(d.final);
  d.preJumps.clear();
  d.intermediates.clear();
  d.timeTrajectory.clear();
}

inline void sampleIntermediateDualSolution(const DualSolution& dualSolution, const scalar_array_t& timeTrajectory,
                                           std::vector<MultiplierCollection>& intermediateDualSolution) {
  // re-sample dual solution
  intermediateDualSolution.clear();
  intermediateDualSolution.reserve(timeTrajectory.size());
  for (const auto& t : timeTrajectory) {
    const auto indexAlpha = LinearInterpolation::timeSegment(t, dualSolution.timeTrajectory);
    intermediateDualSolution.push_back(LinearInterpolation::interpolate(indexAlpha, dualSolution.intermediates));
  }
}

}  // namespace ocs2
