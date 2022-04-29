#pragma once

#include <Eigen/Sparse>

#include <ocs2_core/Types.h>
#include <ocs2_core/thread_support/ThreadPool.h>

#include "ocs2_oc/oc_problem/OcpSize.h"

namespace ocs2 {

/**
 * Calculate the scaling factor D, E and c, and scale the input matrix(vector) H, h and G in place. The scaled matrix(vector) are defined as
 * \f[
 *      \tilde{H} = cDHD, \tilde{h} = cDh,
 *      \tilde{G} = EGD,  \tilde{g} = Eg
 * \f]
 *
 *
 * @param[in, out] H Cost hessian.
 * @param[in, out] h Linear cost term.
 * @param[in, out] G Linear constraint matrix.
 * @param[in] iteration Number of iteration.
 * @param[out] DOut Scaling factor D
 * @param[out] EOut Scaling factor E
 * @param[out] cOut Scaling factor c
 */
void preConditioningSparseMatrixInPlace(Eigen::SparseMatrix<scalar_t>& H, vector_t& h, Eigen::SparseMatrix<scalar_t>& G,
                                        const int iteration, vector_t& DOut, vector_t& EOut, scalar_t& cOut);

/**
 * @brief  Calculate the scaling factor D, E and c, and scale the input dynamics, cost data in place in parallel.
 *
 *
 * @param[in] x0 Initial state
 * @param[in] ocpSize The size of the oc problem.
 * @param[in] iteration Number of iteration.
 * @param[in,  out] dynamics The dynamics array od all time points.
 * @param[in,  out] cost The cost array of all time points.
 * @param[out] DOut Scaling factor D
 * @param[out] EOut Scaling factor E
 * @param[out] scalingVectors Vector representatoin for the identity parts of the dynamics constraints inside the constraint matrix.
 * After scaling, they become arbitrary diagonal matrices. scalingVectors store the diagonal components of this type of matrix for every
 * timestamp.
 * @param[out] cOut Scaling factor c
 * @param[in] threadPool External thread pool.
 * @param[in] H For test only. Can be removed.
 * @param[in] h For test only. Can be removed.
 * @param[in] G For test only. Can be removed.
 */
void preConditioningInPlaceInParallel(const vector_t& x0, const OcpSize& ocpSize, const int iteration,
                                      std::vector<VectorFunctionLinearApproximation>& dynamics,
                                      std::vector<ScalarFunctionQuadraticApproximation>& cost, vector_array_t& DOut, vector_array_t& EOut,
                                      vector_array_t& scalingVectors, scalar_t& cOut, ThreadPool& threadPool,
                                      const Eigen::SparseMatrix<scalar_t>& H, const vector_t& h, const Eigen::SparseMatrix<scalar_t>& G);
/**
 * @brief Scale the dynamics and cost array in place and construct scaling vector array from the given scaling factors E, D and c.
 *
 * @param[in] ocpSize The size of the oc problem.
 * @param[in] D Scaling factor D
 * @param[in] E Scaling factor E
 * @param[in] c Scaling factor c
 * @param[in, out] dynamics Dynamics array
 * @param[in, out] cost Cost array
 * @param[out] scalingVectors Vector representatoin for the identity parts of the dynamics constraints inside the constraint matrix.
 * After scaling, they become arbitrary diagonal matrices. scalingVectors store the diagonal components of this type of matrix for every
 * timestamp.
 */
void scaleDataInPlace(const OcpSize& ocpSize, const vector_t& D, const vector_t& E, const scalar_t c,
                      std::vector<VectorFunctionLinearApproximation>& dynamics, std::vector<ScalarFunctionQuadraticApproximation>& cost,
                      std::vector<vector_t>& scalingVectors);
}  // namespace ocs2