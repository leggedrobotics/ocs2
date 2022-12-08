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

#include "ocs2_slp/pipg/PipgBounds.h"
#include "ocs2_slp/pipg/PipgSettings.h"
#include "ocs2_slp/pipg/PipgSolverStatus.h"

namespace ocs2 {
namespace pipg {

/**
 * First order primal-dual method for solving optimal control problem based on:
 * "Proportional-Integral Projected Gradient Method for Model Predictive Control"
 * https://arxiv.org/abs/2009.06980
 *
 * z := [u_{0}; x_{1}; ...; u_{n}; x_{n+1}].
 *
 * min  0.5 z' H z + z' h
 * s.t. G z = g
 *
 * H = [ R0
 *       *   Q1  P1'
 *       *   P1  R1
 *       *   *   *   Qn  Pn'
 *       *   *   *   Pn  Rn
 *       *   *   *   *   *   Q{n+1}]
 * h = [(P0 x0 + r0); q1; r1 ...; qn; rn; q_{n+1}]
 *
 * G = [-B0  I
 *       *  -A1 -B1   I
 *
 *       *   *   *   -An -Bn  I
 *       D0  0
 *       *   C1  D1   0
 *
 *       *   *   *    Cn  Dn  0]
 * g = [(A0 x0 + b0); b1; ...; bn, -(C0 x0 + e0); -e1; ...; en]
 *
 * For constructing H, h, G, and g, refer to "ocs2_oc/oc_problem/OcpToKkt.h".
 *
 * @param [in] settings : The PIPG settings.
 * @param [in] H : The hessian matrix of the total cost.
 * @param [in] h : The jacobian vector of the total cost.
 * @param [in] G : The jacobian matrix of the constarinst.
 * @param [in] g : The constraints vector.
 * @param [in] EInv : Inverse of the scaling factor E. Used to calculate un-sacled termination criteria.
 * @param [in] pipgBounds : The PipgBounds used to define the primal and dual stepsizes.
 * @param [out] stackedSolution : The concatenated state-input trajectories, z.
 * @return The solver status.
 */
SolverStatus singleThreadPipg(const pipg::Settings& settings, const Eigen::SparseMatrix<scalar_t>& H, const vector_t& h,
                              const Eigen::SparseMatrix<scalar_t>& G, const vector_t& g, const vector_t& EInv, const PipgBounds& pipgBounds,
                              vector_t& stackedSolution);

}  // namespace pipg
}  // namespace ocs2
