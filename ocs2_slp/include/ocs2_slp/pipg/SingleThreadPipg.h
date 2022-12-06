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

#include <Eigen/Sparse>

#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_oc/oc_problem/OcpSize.h>

#include "ocs2_slp/pipg/PipgSettings.h"
#include "ocs2_slp/pipg/PipgSolverStatus.h"

namespace ocs2 {
namespace pipg {

/*
 * First order primal-dual method for solving optimal control problem based on:
 * "Proportional-Integral Projected Gradient Method for Model Predictive Control"
 * https://arxiv.org/abs/2009.06980
 */
SolverStatus singleThreadPipg(const pipg::Settings& settings, const Eigen::SparseMatrix<scalar_t>& H, const vector_t& h,
                              const Eigen::SparseMatrix<scalar_t>& G, const vector_t& g, const vector_t& EInv, const scalar_t mu,
                              const scalar_t lambda, const scalar_t sigma, vector_t& stackedSolution);

/**
 * Deserializes the stacked solution to state-input trajecotries.
 *
 * @param[in] ocpSize : Optimal control problem sizes.
 * @param[in] stackedSolution : Defined as [u[0], x[1], ..., u[N-1], x[N]].
 * @param[in] x0 : The initial state.
 * @param[out] xTrajectory : State tarjectory.
 * @param[out] uTrajectory : Input trajecotry.
 */
void unpackSolution(const OcpSize& ocpSize, const vector_t& stackedSolution, const vector_t x0, vector_array_t& xTrajectory,
                    vector_array_t& uTrajectory);

/**
 * Serializes the state-input trajecotries.
 *
 * @param[in] xTrajectory : State tarjectory
 * @param[in] UTrajectory : Input trajecotry
 * @param[out] stackedSolution : [u[0], x[1], ..., u[N-1], x[N]]
 */
void packSolution(const vector_array_t& xTrajectory, const vector_array_t& uTrajectory, vector_t& stackedSolution);

}  // namespace pipg
}  // namespace ocs2
