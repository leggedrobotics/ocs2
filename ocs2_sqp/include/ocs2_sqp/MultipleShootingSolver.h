//
// Created by rgrandia on 09.11.20.
//

#pragma once
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/constraint/PenaltyBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_oc/oc_solver/SolverBase.h>

#include <ocs2_sqp/HpipmInterface.h>
#include <iostream>

namespace ocs2 {

struct MultipleShootingSolverSettings {
  scalar_t dt = 0.01;  // user-defined time discretization
  size_t n_state = 0;
  size_t n_input = 0;
  size_t sqpIteration = 1;
  scalar_t deltaTol = 1e-6;

  // Inequality penalty method
  scalar_t inequalityConstraintMu = 0.0;
  scalar_t inequalityConstraintDelta = 1e-6;

  bool qr_decomp = true;  // Only meaningful if the system is constrained. True to use QR decomposiion, False to use lg <= Cx+Du+e <= ug
  bool printSolverStatus = false;
  bool printSolverStatistics = false;
  bool printModeScheduleDebug = false;
};

class MultipleShootingSolver : public SolverBase {
 public:
  MultipleShootingSolver(MultipleShootingSolverSettings settings, const SystemDynamicsBase* systemDynamicsPtr,
                         const CostFunctionBase* costFunctionPtr, const ConstraintBase* constraintPtr = nullptr,
                         const CostFunctionBase* terminalCostFunctionPtr = nullptr);

  ~MultipleShootingSolver() override;

  // TODO
  void reset() override;
  scalar_t getFinalTime() const override { return primalSolution_.timeTrajectory_.back(); };  // horizon is [t0, T] return T;
  // fill primal solution after solving the problem.
  void getPrimalSolution(scalar_t finalTime, PrimalSolution* primalSolutionPtr) const override { *primalSolutionPtr = primalSolution_; }
  void printPrimalSolution() const {
    for (int i = 0; i < primalSolution_.timeTrajectory_.size(); i++) {
      std::cout << "time: " << primalSolution_.timeTrajectory_[i] << "\t state: " << primalSolution_.stateTrajectory_[i].transpose()
                << "\t input: " << primalSolution_.inputTrajectory_[i].transpose() << std::endl;
    }
  }

  // Maybe Ignore
  const PerformanceIndex& getPerformanceIndeces() const override { return performanceIndex_; }
  size_t getNumIterations() const override { return 1; }
  const std::vector<PerformanceIndex>& getIterationsLog() const override { return performanceIndeces_; };
  const scalar_array_t& getPartitioningTimes() const override { return partitionTime_; };
  ScalarFunctionQuadraticApproximation getValueFunction(scalar_t time, const vector_t& state) const override {
    return ScalarFunctionQuadraticApproximation::Zero(0, 0);
  };
  vector_t getStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t& state) const override { return vector_t::Zero(0); }
  void rewindOptimizer(size_t firstIndex) override{};
  const unsigned long long int& getRewindCounter() const override {
    throw std::runtime_error("[MultipleShootingSolver] no rewind counter");
  };

  static scalar_array_t timeDiscretizationWithEvents(scalar_t initTime, scalar_t finalTime, scalar_t dt, const scalar_array_t& eventTimes,
                                                     scalar_t eventDelta);

 private:
  std::string getBenchmarkingInformation() const;
  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes) override;

  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes,
               const std::vector<ControllerBase*>& controllersPtrStock) override {
    runImpl(initTime, initState, finalTime, partitioningTimes);
  }

  std::vector<vector_t> initializeStateTrajectory(const vector_t& initState, const scalar_array_t& timeDiscretization, int N) const;
  std::vector<vector_t> initializeInputTrajectory(const scalar_array_t& timeDiscretization, int N) const;
  void setupCostDynamicsEqualityConstraint(SystemDynamicsBase& systemDynamics, CostFunctionBase& costFunction,
                                           ConstraintBase* constraintPtr, CostFunctionBase* terminalCostFunctionPtr,
                                           const scalar_array_t& time, const std::vector<ocs2::vector_t>& x,
                                           const std::vector<ocs2::vector_t>& u);
  std::pair<std::vector<ocs2::vector_t>, std::vector<ocs2::vector_t>> getOCPSolution(const vector_t& delta_x0);

  MultipleShootingSolverSettings settings_;
  std::unique_ptr<SystemDynamicsBase> systemDynamicsPtr_;
  std::unique_ptr<CostFunctionBase> costFunctionPtr_;
  std::unique_ptr<ConstraintBase> constraintPtr_;
  std::unique_ptr<CostFunctionBase> terminalCostFunctionPtr_;
  std::unique_ptr<PenaltyBase> penaltyPtr_;

  PrimalSolution primalSolution_;

  HpipmInterface hpipmInterface_;

  // LQ approximation
  std::vector<VectorFunctionLinearApproximation> dynamics_;
  std::vector<ScalarFunctionQuadraticApproximation> cost_;
  std::vector<VectorFunctionLinearApproximation> constraints_;

  // Benchmarking
  size_t totalNumIterations_;
  benchmark::RepeatedTimer initializationTimer_;
  benchmark::RepeatedTimer linearQuadraticApproximationTimer_;
  benchmark::RepeatedTimer solveQpTimer_;
  benchmark::RepeatedTimer computeControllerTimer_;

  // Unused : just to implement the interface
  PerformanceIndex performanceIndex_;
  std::vector<PerformanceIndex> performanceIndeces_ = {PerformanceIndex()};
  scalar_array_t partitionTime_;
};

}  // namespace ocs2