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

#include <vector>

#include <ocs2_core/Types.h>

namespace ocs2 {
/**
 * Size of the optimal control problem to be solved with the HpipmInterface
 *
 * The number of stages (N) is defined as the number of transitions, the number of nodes, including terminal node, is therefore (N+1).
 *
 * For example, a problem with:
 * (t0, x0) -> dynamics driven by u0 ->  (t1, x1) -> dynamics driven by u1 -> (t2, x2)
 * Has 2 stages, and 3 nodes.
 *
 * HPIPM requires all sizes to be declared per node. All vectors therefore need to be of size N+1.
 * For the optimal control problems we are currently solving, we therefore set numInput(N+1) = 0.
 */
struct OcpSize {
  int numStages;                            // Number of stages (N), all vectors below must be of size N+1
  std::vector<int> numInputs;               // Number of inputs
  std::vector<int> numStates;               // Number of states
  std::vector<int> numInputBoxConstraints;  // Number of input box inequality constraints
  std::vector<int> numStateBoxConstraints;  // Number of state box inequality constraints
  std::vector<int> numIneqConstraints;      // Number of general inequality constraints
  std::vector<int> numInputBoxSlack;        // Number of slack variables for input box inequalities
  std::vector<int> numStateBoxSlack;        // Number of slack variables for state box inequalities
  std::vector<int> numIneqSlack;            // Number of slack variables for general inequalities

  /** Constructor for N stages with constant state and inputs and without constraints */
  explicit OcpSize(int N = 0, int nx = 0, int nu = 0)
      : numStages(N),
        numStates(N + 1, nx),
        numInputs(N + 1, nu),
        numInputBoxConstraints(N + 1, 0),
        numStateBoxConstraints(N + 1, 0),
        numIneqConstraints(N + 1, 0),
        numInputBoxSlack(N + 1, 0),
        numStateBoxSlack(N + 1, 0),
        numIneqSlack(N + 1, 0) {
    numInputs.back() = 0;
  }
};

bool operator==(const OcpSize& lhs, const OcpSize& rhs) noexcept;

/**
 * Extract sizes based on the problem data
 *
 * @param dynamics : Linearized approximation of the discrete dynamics.
 * @param cost : Quadratic approximation of the cost.
 * @param constraints : Linearized approximation of constraints, all constraints are mapped to inequality constraints in HPIPM.
 * @return Derived sizes
 */
OcpSize extractSizesFromProblem(const std::vector<VectorFunctionLinearApproximation>& dynamics,
                                const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                                const std::vector<VectorFunctionLinearApproximation>* constraints);

}  // namespace ocs2