#pragma once

#include <vector>

#include <ocs2_core/Types.h>

namespace ocs2 {
namespace pipg {

struct OcpSize {
  int numStages;                        // Number of stages (N), all vectors below must be of size N+1
  std::vector<int> numInputs;           // Number of inputs
  std::vector<int> numStates;           // Number of states
  std::vector<int> numIneqConstraints;  // Number of general inequality constraints

  /** Constructor for N stages with constant state and inputs and without constraints */
  explicit OcpSize(int N = 0, int nx = 0, int nu = 0)
      : numStages(N), numStates(N + 1, nx), numInputs(N + 1, nu), numIneqConstraints(N + 1, 0) {
    numInputs.back() = 0;
  }
};

bool operator==(const OcpSize& lhs, const OcpSize& rhs) noexcept;

/**
 * Extract sizes based on the problem data
 *
 * @param dynamics : Linearized approximation of the discrete dynamics.
 * @param cost : Quadratic approximation of the cost.
 * @param constraints : Linearized approximation of constraints.
 * @return Derived sizes
 */
OcpSize extractSizesFromProblem(const std::vector<VectorFunctionLinearApproximation>& dynamics,
                                const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                                const std::vector<VectorFunctionLinearApproximation>* constraints);

}  // namespace pipg
}  // namespace ocs2