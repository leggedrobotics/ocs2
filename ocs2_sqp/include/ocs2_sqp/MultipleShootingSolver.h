//
// Created by rgrandia on 09.11.20.
//

#pragma once

#include <ocs2_oc/oc_solver/Solver_BASE.h>

namespace ocs2 {

struct MultipleShootingSolverSettings {
  int N = 10;
  int nx
};

class MultipleShootingSolver : public Solver_BASE {
 public:
  /**
   * Pass dynamics costs etc. See what you need.
   */
  MultipleShootingSolver(MultipleShootingSolverSettings settings, const SystemDynamicsBase* systemDynamicsPtr, const CostFunctionBase* costFunctionPtr);

  ~MultipleShootingSolver() override = default;

  // TODO
  void reset() override;
  scalar_t getFinalTime() const override {return primalSolution_.timeTrajectory_.back(); }; // horizon is [t0, T] return T;
  // fill primal solution after solving the problem.
  void getPrimalSolution(scalar_t finalTime, PrimalSolution* primalSolutionPtr) const override { primalSolutionPtr = &primalSolution_; }

  // Maybe Ignore
  const PerformanceIndex& getPerformanceIndeces() const override { return performanceIndex_; }
  size_t getNumIterations() const override { return 1; }
  const std::vector<PerformanceIndex>& getIterationsLog() const override { return performanceIndeces_; };
  const scalar_array_t& getPartitioningTimes() const override { return partitionTime_; };
  scalar_t getValueFunction(scalar_t time, const vector_t& state) const override { return 0.0; };
  void getValueFunctionStateDerivative(scalar_t time, const vector_t& state, vector_t& Vx) const override{};
  void getStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t& state, vector_t& nu) const override{};
  void rewindOptimizer(size_t firstIndex) {};
  const unsigned long long int& getRewindCounter() const { 0; };

 private:
  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes) override;

  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes,
               const std::vector<ControllerBase*>& controllersPtrStock) override { runImpl(initTime, initState, finalTime, partitioningTimes); }

  MultipleShootingSolverSettings settings_;
  std::unique_ptr<SystemDynamicsBase> systemDynamicsPtr_;
  std::unique_ptr<CostFunctionBase> costFunctionPtr_;

  PrimalSolution primalSolution_;


  // Unused : just to implement the interface
  PerformanceIndex performanceIndex_;
  std::vector<PerformanceIndex> performanceIndeces_ = {PerformanceIndex()};
  scalar_array_t partitionTime_;
};

}  // namespace ocs2