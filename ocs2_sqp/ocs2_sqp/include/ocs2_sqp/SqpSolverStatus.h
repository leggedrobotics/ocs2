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

#include <string>

#include <ocs2_core/Types.h>
#include <ocs2_oc/oc_data/PerformanceIndex.h>
#include <ocs2_oc/search_strategy/FilterLinesearch.h>

namespace ocs2 {
namespace sqp {

/** Different types of convergence */
enum class Convergence { FALSE, ITERATIONS, STEPSIZE, METRICS, PRIMAL };

/** Struct to contain the result and logging data of the stepsize computation */
struct StepInfo {
  // Step size and type
  scalar_t stepSize = 0.0;
  FilterLinesearch::StepType stepType = FilterLinesearch::StepType::UNKNOWN;

  // Step in primal variables
  scalar_t dx_norm = 0.0;  // norm of the state trajectory update
  scalar_t du_norm = 0.0;  // norm of the input trajectory update

  // Performance result after the step
  PerformanceIndex performanceAfterStep;
  scalar_t totalConstraintViolationAfterStep;  // constraint metric used in the line search
};

/** Transforms sqp::Convergence to string */
inline std::string toString(const Convergence& convergence) {
  switch (convergence) {
    case Convergence::ITERATIONS:
      return "Maximum number of iterations reached";
    case Convergence::STEPSIZE:
      return "Step size below minimum";
    case Convergence::METRICS:
      return "Cost decrease and constraint satisfaction below tolerance";
    case Convergence::PRIMAL:
      return "Primal update below tolerance";
    case Convergence::FALSE:
    default:
      return "Not Converged";
  }
}

}  // namespace sqp
}  // namespace ocs2
