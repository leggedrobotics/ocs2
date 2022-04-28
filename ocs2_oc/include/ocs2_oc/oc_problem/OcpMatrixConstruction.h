#pragma once

#include <Eigen/Sparse>

#include <ocs2_core/Types.h>

#include "ocs2_oc/oc_problem/OcpSize.h"

namespace ocs2 {

/**
 * @brief Get liner constraint matrix G from the dynamics, cost and constraint arrays.
 *
 * @param ocpSize: The size of optimal control problem.
 * @param x0: Initial state.
 * @param dynamics: Dynamics array.
 * @param constraints: Constraints array.
 * @param scalingVectorsPtr: Vector representatoin for the identity parts of the dynamics constraints inside the constraint matrix. After
 * scaling, they become arbitrary diagonal matrices. Pass nullptr to get them filled with identity matrices.
 * @param res: The resulting constraints approxmation.
 */
void getConstraintMatrix(const OcpSize& ocpSize, const vector_t& x0, const std::vector<VectorFunctionLinearApproximation>& dynamics,
                         const std::vector<VectorFunctionLinearApproximation>* constraints, const vector_array_t* scalingVectorsPtr,
                         VectorFunctionLinearApproximation& res);

void getConstraintMatrixSparse(const OcpSize& ocpSize, const vector_t& x0, const std::vector<VectorFunctionLinearApproximation>& dynamics,
                               const std::vector<VectorFunctionLinearApproximation>* constraints, const vector_array_t* scalingVectorsPtr,
                               Eigen::SparseMatrix<scalar_t>& G, vector_t& g);

void getCostMatrix(const OcpSize& ocpSize, const vector_t& x0, const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                   ScalarFunctionQuadraticApproximation& res);

void getCostMatrixSparse(const OcpSize& ocpSize, const vector_t& x0, const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                         Eigen::SparseMatrix<scalar_t>& H, vector_t& h);
}  // namespace ocs2