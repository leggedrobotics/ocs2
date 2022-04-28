#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/thread_support/ThreadPool.h>
#include <ocs2_oc/oc_problem/OcpSize.h>

namespace ocs2 {
namespace pipg {

vector_t hessianAbsRowSum(const OcpSize& ocpSize, const std::vector<ScalarFunctionQuadraticApproximation>& cost);

vector_t GGTAbsRowSumInParallel(const OcpSize& ocpSize, const std::vector<VectorFunctionLinearApproximation>& dynamics,
                                const std::vector<VectorFunctionLinearApproximation>* constraints, const vector_array_t* scalingVectorsPtr,
                                ThreadPool& threadPool);
}  // namespace pipg
}  // namespace ocs2