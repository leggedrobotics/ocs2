#include "ocs2_pipg/OcpSize.h"

namespace ocs2 {
namespace pipg {

bool operator==(const OcpSize& lhs, const OcpSize& rhs) noexcept {
  // use && instead of &= to enable short-circuit evaluation
  bool same = lhs.numStages == rhs.numStages;
  same = same && (lhs.numInputs == rhs.numInputs);
  same = same && (lhs.numStates == rhs.numStates);
  same = same && (lhs.numIneqConstraints == rhs.numIneqConstraints);
  return same;
}

OcpSize extractSizesFromProblem(const std::vector<VectorFunctionLinearApproximation>& dynamics,
                                const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                                const std::vector<VectorFunctionLinearApproximation>* constraints) {
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

  return problemSize;
}

}  // namespace pipg
}  // namespace ocs2