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

#include "hpipm_catkin/OcpSize.h"

namespace ocs2 {
namespace hpipm_interface {

bool operator==(const OcpSize& lhs, const OcpSize& rhs) noexcept {
  // use && instead of &= to enable short-circuit evaluation
  bool same = lhs.numStages == rhs.numStages;
  same = same && (lhs.numInputs == rhs.numInputs);
  same = same && (lhs.numStates == rhs.numStates);
  same = same && (lhs.numInputBoxConstraints == rhs.numInputBoxConstraints);
  same = same && (lhs.numStateBoxConstraints == rhs.numStateBoxConstraints);
  same = same && (lhs.numIneqConstraints == rhs.numIneqConstraints);
  same = same && (lhs.numInputBoxSlack == rhs.numInputBoxSlack);
  same = same && (lhs.numStateBoxSlack == rhs.numStateBoxSlack);
  same = same && (lhs.numIneqSlack == rhs.numIneqSlack);
  return same;
}

OcpSize extractSizesFromProblem(const std::vector<VectorFunctionLinearApproximation>& dynamics,
                                const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                                const std::vector<VectorFunctionLinearApproximation>* constraints,
                                const std::vector<VectorFunctionLinearApproximation>* ineqConstraints,
                                bool useSlack) {
  const int numStages = dynamics.size();

  OcpSize problemSize(dynamics.size());

  // State inputs
  for (int k = 0; k < numStages; k++) {
    problemSize.numStates[k] = dynamics[k].dfdx.cols();
    problemSize.numInputs[k] = dynamics[k].dfdu.cols();
  }
  problemSize.numStates[numStages] = dynamics[numStages - 1].dfdx.rows();
  problemSize.numInputs[numStages] = 0;

  // Constraints
  if (constraints != nullptr) {
    for (int k = 0; k < numStages + 1; k++) {
      problemSize.numIneqConstraints[k] = (*constraints)[k].f.size();
    }
  }
  if (ineqConstraints != nullptr) {
    for (int k = 0; k < numStages + 1; k++) {
      problemSize.numIneqConstraints[k] += (*ineqConstraints)[k].f.size();
    }
  }

  // Slack variables
  if (useSlack) {
    for (int k = 0; k < numStages + 1; k++) {
      problemSize.numIneqSlack[k] = problemSize.numIneqConstraints[k];
    }
  }

  return problemSize;
}

}  // namespace hpipm_interface
}  // namespace ocs2
