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

#include "ocs2_sqp/SqpLogging.h"

#include <iomanip>

namespace ocs2 {
namespace sqp {

std::ostream& operator<<(std::ostream& stream, const LogEntry& logEntry) {
  const std::string delim = ", ";
  const std::string lineEnd = "\n";
  // clang-format off
  stream  << std::setprecision(16) // print decimals up to machine epsilon of double (~10^-16)
          << logEntry.problemNumber << delim
          << logEntry.time << delim
          << logEntry.iteration << delim
          << logEntry.linearQuadraticApproximationTime << delim
          << logEntry.solveQpTime << delim
          << logEntry.linesearchTime << delim
          << logEntry.baselinePerformanceIndex.merit << delim
          << logEntry.baselinePerformanceIndex.dynamicsViolationSSE << delim
          << logEntry.baselinePerformanceIndex.equalityConstraintsSSE << delim
          << logEntry.totalConstraintViolationBaseline << delim
          << logEntry.stepInfo.stepSize << delim
          << toString(logEntry.stepInfo.stepType) << delim
          << logEntry.stepInfo.dx_norm << delim
          << logEntry.stepInfo.du_norm << delim
          << logEntry.stepInfo.performanceAfterStep.merit << delim
          << logEntry.stepInfo.performanceAfterStep.dynamicsViolationSSE << delim
          << logEntry.stepInfo.performanceAfterStep.equalityConstraintsSSE << delim
          << logEntry.stepInfo.totalConstraintViolationAfterStep << delim
          << toString(logEntry.convergence) << lineEnd;
  // clang-format on
  return stream;
}

std::string logHeader() {
  const std::string delim = ", ";
  const std::string lineEnd = "\n";
  std::stringstream stream;
  // clang-format off
  stream  << "problemNumber" << delim
          << "time" << delim
          << "iteration" << delim
          << "linearQuadraticApproximationTime" << delim
          << "solveQpTime" << delim
          << "linesearchTime" << delim
          << "baselinePerformanceIndex/merit" << delim
          << "baselinePerformanceIndex/dynamicsViolationSSE" << delim
          << "baselinePerformanceIndex/equalityConstraintsSSE" << delim
          << "totalConstraintViolationBaseline" << delim
          << "stepSize" << delim
          << "stepType" << delim
          << "dxNorm" << delim
          << "duNorm" << delim
          << "performanceAfterStep/merit" << delim
          << "performanceAfterStep/dynamicsViolationSSE" << delim
          << "performanceAfterStep/equalityConstraintsSSE" << delim
          << "totalConstraintViolationAfterStep" << delim
          << "convergence" << lineEnd;
  // clang-format on
  return stream.str();
}

}  // namespace sqp
}  // namespace ocs2
