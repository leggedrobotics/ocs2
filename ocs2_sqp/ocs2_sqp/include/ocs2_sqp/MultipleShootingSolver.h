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

#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/constraint/PenaltyBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_core/initialization/SystemOperatingTrajectoriesBase.h>
#include <ocs2_core/integration/SensitivityIntegrator.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_core/misc/ThreadPool.h>
#include <ocs2_oc/oc_solver/SolverBase.h>

#include <hpipm_catkin/HpipmInterface.h>

#include "ocs2_sqp/MultipleShootingSettings.h"

namespace ocs2 {

class MultipleShootingSolver : public SolverBase {
 public:
  using Settings = multiple_shooting::Settings;

  /**
   * Constructor
   *
   * @param settings : settings for the multiple shooting solver.
   * @param systemDynamicsPtr : The system dynamics.
   * @param costFunctionPtr : The cost function used for the intermediate costs.
   * @param constraintPtr : The system constraint function.
   * @param terminalCostPtr : The cost function used for the terminal (=at the end of the horizon) costs.
   * @param operatingTrajectoriesPtr : The operating trajectories of system used for initialization.
   */
  MultipleShootingSolver(Settings settings, const SystemDynamicsBase* systemDynamicsPtr, const CostFunctionBase* costFunctionPtr,
                         const ConstraintBase* constraintPtr = nullptr, const CostFunctionBase* terminalCostFunctionPtr = nullptr,
                         const SystemOperatingTrajectoriesBase* operatingTrajectoriesPtr = nullptr);

  ~MultipleShootingSolver() override;

  void reset() override;

  scalar_t getFinalTime() const override { return primalSolution_.timeTrajectory_.back(); };

  void getPrimalSolution(scalar_t finalTime, PrimalSolution* primalSolutionPtr) const override { *primalSolutionPtr = primalSolution_; }

  size_t getNumIterations() const override { return totalNumIterations_; }

  const PerformanceIndex& getPerformanceIndeces() const override { return getIterationsLog().back(); };

  const std::vector<PerformanceIndex>& getIterationsLog() const override;

  /** TODO */
  ScalarFunctionQuadraticApproximation getValueFunction(scalar_t time, const vector_t& state) const override {
    return ScalarFunctionQuadraticApproximation::Zero(0, 0);
  };

  /** TODO */
  vector_t getStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t& state) const override { return vector_t::Zero(0); }

  // Irrelevant baseclass stuff
  void rewindOptimizer(size_t firstIndex) override{};
  const unsigned long long int& getRewindCounter() const override {
    throw std::runtime_error("[MultipleShootingSolver] no rewind counter");
  };
  const scalar_array_t& getPartitioningTimes() const override { return partitionTime_; };

 private:
  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes) override;

  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes,
               const std::vector<ControllerBase*>& controllersPtrStock) override {
    runImpl(initTime, initState, finalTime, partitioningTimes);
  }

  /** Run a task in parallel with settings.nThreads */
  void runParallel(std::function<void(int)> taskFunction);

  /** Get profiling information as a string */
  std::string getBenchmarkingInformation() const;

  /** Returns initial guess for the state trajectory */
  vector_array_t initializeStateTrajectory(const vector_t& initState, const scalar_array_t& timeDiscretization, int N) const;

  /** Returns initial guess for the input trajectory */
  vector_array_t initializeInputTrajectory(const scalar_array_t& timeDiscretization, const vector_array_t& stateTrajectory, int N) const;

  /** Creates QP around t, x(t), u(t). Returns performance metrics at the current {t, x(t), u(t)} */
  PerformanceIndex setupQuadraticSubproblem(const scalar_array_t& time, const vector_t& initState, const vector_array_t& x,
                                            const vector_array_t& u);

  /** Computes only the performance metrics at the current {t, x(t), u(t)} */
  PerformanceIndex computePerformance(const scalar_array_t& time, const vector_t& initState, const vector_array_t& x,
                                      const vector_array_t& u);

  /** Returns solution of the QP subproblem in delta coordinates: {delta_x(t), delta_u(t)} */
  std::pair<vector_array_t, vector_array_t> getOCPSolution(const vector_t& delta_x0);

  /** Compute 2-norm of the trajectory: sqrt(sum_i v[i]^2)  */
  static scalar_t trajectoryNorm(const vector_array_t& v);

  /** Decides on the step to take and overrides given trajectories {x(t), u(t)} <- {x(t) + a*dx(t), u(t) + a*du(t)} */
  bool takeStep(const PerformanceIndex& baseline, const scalar_array_t& timeDiscretization, const vector_t& initState,
                const vector_array_t& dx, const vector_array_t& du, vector_array_t& x, vector_array_t& u);

  // Problem definition
  Settings settings_;
  DynamicsDiscretizer discretizer_;
  DynamicsSensitivityDiscretizer sensitivityDiscretizer_;
  std::vector<std::unique_ptr<SystemDynamicsBase>> systemDynamicsPtr_;
  std::vector<std::unique_ptr<CostFunctionBase>> costFunctionPtr_;
  std::vector<std::unique_ptr<ConstraintBase>> constraintPtr_;
  std::unique_ptr<CostFunctionBase> terminalCostFunctionPtr_;
  std::unique_ptr<SystemOperatingTrajectoriesBase> operatingTrajectoriesPtr_;
  std::unique_ptr<PenaltyBase> penaltyPtr_;

  // Threading
  std::unique_ptr<ThreadPool> threadPoolPtr_;

  // Solution
  PrimalSolution primalSolution_;

  // Solver interface
  HpipmInterface hpipmInterface_;

  // LQ approximation
  std::vector<VectorFunctionLinearApproximation> dynamics_;
  std::vector<ScalarFunctionQuadraticApproximation> cost_;
  std::vector<VectorFunctionLinearApproximation> constraints_;

  // Iteration performance log
  std::vector<PerformanceIndex> performanceIndeces_;

  // Benchmarking
  size_t totalNumIterations_;
  benchmark::RepeatedTimer initializationTimer_;
  benchmark::RepeatedTimer linearQuadraticApproximationTimer_;
  benchmark::RepeatedTimer solveQpTimer_;
  benchmark::RepeatedTimer linesearchTimer_;
  benchmark::RepeatedTimer computeControllerTimer_;

  scalar_array_t partitionTime_ = {};
};

}  // namespace ocs2