//
// Created by rgrandia on 09.11.20.
//

#pragma once

#include <iostream>

#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/constraint/PenaltyBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_core/initialization/SystemOperatingTrajectoriesBase.h>
#include <ocs2_core/integration/SensitivityIntegrator.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_oc/oc_solver/SolverBase.h>

#include <ocs2_sqp/HpipmInterface.h>

namespace ocs2 {

struct MultipleShootingSolverSettings {
  scalar_t dt = 0.01;  // user-defined time discretization
  size_t n_state = 0;
  size_t n_input = 0;
  size_t sqpIteration = 1;
  scalar_t deltaTol = 1e-6;

  // Discretization method
  SensitivityIntegratorType integratorType = SensitivityIntegratorType::RK2;

  // Inequality penalty method
  scalar_t inequalityConstraintMu = 0.0;
  scalar_t inequalityConstraintDelta = 1e-6;

  bool qr_decomp = true;  // Only meaningful if the system is constrained. True to use QR decomposiion, False to use lg <= Cx+Du+e <= ug
  bool printSolverStatus = false;      // Print HPIPM status after solving the QP subproblem
  bool printSolverStatistics = false;  // Print benchmarking of the multiple shooting method
  bool printLinesearch = false;        // Print linesearch information
};

class MultipleShootingSolver : public SolverBase {
 public:
  MultipleShootingSolver(MultipleShootingSolverSettings settings, const SystemDynamicsBase* systemDynamicsPtr,
                         const CostFunctionBase* costFunctionPtr, const ConstraintBase* constraintPtr = nullptr,
                         const CostFunctionBase* terminalCostFunctionPtr = nullptr,
                         const SystemOperatingTrajectoriesBase* operatingTrajectoriesPtr = nullptr);

  ~MultipleShootingSolver() override;

  void reset() override;

  scalar_t getFinalTime() const override { return primalSolution_.timeTrajectory_.back(); };  // horizon is [t0, T] return T;
  void getPrimalSolution(scalar_t finalTime, PrimalSolution* primalSolutionPtr) const override { *primalSolutionPtr = primalSolution_; }

  size_t getNumIterations() const override { return totalNumIterations_; }
  const PerformanceIndex& getPerformanceIndeces() const override { return getIterationsLog().back(); };
  const std::vector<PerformanceIndex>& getIterationsLog() const override;

  /** Decides on time discretization along the horizon */
  static scalar_array_t timeDiscretizationWithEvents(scalar_t initTime, scalar_t finalTime, scalar_t dt, const scalar_array_t& eventTimes,
                                                     scalar_t eventDelta);

  /** Irrelevant baseclass stuff */
  void rewindOptimizer(size_t firstIndex) override{};
  const unsigned long long int& getRewindCounter() const override {
    throw std::runtime_error("[MultipleShootingSolver] no rewind counter");
  };
  const scalar_array_t& getPartitioningTimes() const override { return partitionTime_; };
  ScalarFunctionQuadraticApproximation getValueFunction(scalar_t time, const vector_t& state) const override {
    return ScalarFunctionQuadraticApproximation::Zero(0, 0);
  };
  vector_t getStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t& state) const override { return vector_t::Zero(0); }

 private:
  /** Get profiling information as a string */
  std::string getBenchmarkingInformation() const;

  /** Entrypoint for this solver, called by the base class */
  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes) override;
  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes,
               const std::vector<ControllerBase*>& controllersPtrStock) override {
    runImpl(initTime, initState, finalTime, partitioningTimes);
  }

  /** Returns initial guess for the state trajectory */
  vector_array_t initializeStateTrajectory(const vector_t& initState, const scalar_array_t& timeDiscretization, int N) const;

  /** Returns initial guess for the input trajectory */
  vector_array_t initializeInputTrajectory(const scalar_array_t& timeDiscretization, const vector_array_t& stateTrajectory, int N) const;

  /** Creates QP around t, x, u. Returns performance metrics at the current {t, x, u} */
  PerformanceIndex setupQuadraticSubproblem(SystemDynamicsBase& systemDynamics, CostFunctionBase& costFunction,
                                            ConstraintBase* constraintPtr, CostFunctionBase* terminalCostFunctionPtr,
                                            const scalar_array_t& time, const vector_array_t& x, const vector_array_t& u);

  /** Computes only the performance metrics at the current {t, x, u} */
  PerformanceIndex computePerformance(SystemDynamicsBase& systemDynamics, CostFunctionBase& costFunction, ConstraintBase* constraintPtr,
                                      CostFunctionBase* terminalCostFunctionPtr, const scalar_array_t& time, const vector_array_t& x,
                                      const vector_array_t& u);

  /** Returns solution of the QP subproblem in delta coordinates: {delta_x, delta_u} */
  std::pair<vector_array_t, vector_array_t> getOCPSolution(const vector_t& delta_x0);

  /** Decides on the step to take and overrides given trajectories {x, u} <- {x + a*dx, u + a*du} */
  bool takeStep(const PerformanceIndex& baseline, const scalar_array_t& timeDiscretization, const vector_array_t& dx,
                const vector_array_t& du, vector_array_t& x, vector_array_t& u);

  // Problem definition
  MultipleShootingSolverSettings settings_;
  DynamicsDiscretizer discretizer_;
  DynamicsSensitivityDiscretizer sensitivityDiscretizer_;
  std::unique_ptr<SystemDynamicsBase> systemDynamicsPtr_;
  std::unique_ptr<CostFunctionBase> costFunctionPtr_;
  std::unique_ptr<ConstraintBase> constraintPtr_;
  std::unique_ptr<CostFunctionBase> terminalCostFunctionPtr_;
  std::unique_ptr<SystemOperatingTrajectoriesBase> operatingTrajectoriesPtr_;
  std::unique_ptr<PenaltyBase> penaltyPtr_;

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
  benchmark::RepeatedTimer computeControllerTimer_;

  scalar_array_t partitionTime_ = {};
};

}  // namespace ocs2