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

#include <string>

#include <Eigen/Sparse>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_core/thread_support/ThreadPool.h>
#include <ocs2_oc/oc_problem/OcpSize.h>

#include "ocs2_slp/pipg/PipgBounds.h"
#include "ocs2_slp/pipg/PipgSettings.h"
#include "ocs2_slp/pipg/PipgSolverStatus.h"

namespace ocs2 {

/*
 * First order primal-dual method for solving optimal control problem based on:
 * "Proportional-Integral Projected Gradient Method for Model Predictive Control"
 * https://arxiv.org/abs/2009.06980
 */
class PipgSolver {
 public:
  /**
   * Constructor.
   * @param[in] Settings: PIPG setting
   */
  explicit PipgSolver(pipg::Settings settings);

  /**
   * Solve the optimal control in parallel.
   *
   * @param [in] threadPool : The external thread pool.
   * @param [in] x0 : Initial state
   * @param [in] dynamics : Dynamics array.
   * @param [in] cost : Cost array.
   * @param [in] constraints : Constraints array. Pass nullptr for an unconstrained problem.
   * @param [in] scalingVectors : Vector representation for the identity parts of the dynamics inside the constraint matrix. After scaling,
   *                              they become arbitrary diagonal matrices. Pass nullptr to get them filled with identity matrices.
   * @param [in] EInv : Inverse of the scaling factor E. Used to calculate un-sacled termination criteria.
   * @param [in] pipgBounds : The PipgBounds used to define the primal and dual stepsizes.
   * @param [out] xTrajectory : The optimized state trajectory.
   * @param [out] uTrajectory : The optimized input trajectory.
   * @return The solver status.
   */
  pipg::SolverStatus solve(ThreadPool& threadPool, const vector_t& x0, std::vector<VectorFunctionLinearApproximation>& dynamics,
                           const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                           const std::vector<VectorFunctionLinearApproximation>* constraints, const vector_array_t& scalingVectors,
                           const vector_array_t* EInv, const pipg::PipgBounds& pipgBounds, vector_array_t& xTrajectory,
                           vector_array_t& uTrajectory);

  void resize(const OcpSize& size);

  int getNumDecisionVariables() const { return numDecisionVariables_; }
  int getNumDynamicsConstraints() const { return numDynamicsConstraints_; }

  const OcpSize& size() const { return ocpSize_; }
  const pipg::Settings& settings() const { return settings_; }

 private:
  void verifySizes(const std::vector<VectorFunctionLinearApproximation>& dynamics,
                   const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                   const std::vector<VectorFunctionLinearApproximation>* constraints) const;

  void verifyOcpSize(const OcpSize& ocpSize) const;

  // Settings
  const pipg::Settings settings_;

  // Problem size
  OcpSize ocpSize_;
  int numDecisionVariables_;
  int numDynamicsConstraints_;

  // Data buffer for parallelized PIPG
  vector_array_t X_, W_, V_, U_;
  vector_array_t XNew_, UNew_, WNew_;
};

}  // namespace ocs2
