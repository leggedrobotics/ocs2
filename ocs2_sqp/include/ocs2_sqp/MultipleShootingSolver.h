//
// Created by rgrandia on 09.11.20.
//

#pragma once
#include <iostream>
#include <ocs2_oc/oc_solver/Solver_BASE.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>

#include <blasfeo_d_aux_ext_dep.h>
extern "C"
{
#include <hpipm_d_ocp_qp_ipm.h>
#include <hpipm_d_ocp_qp_dim.h>
#include <hpipm_d_ocp_qp.h>
#include <hpipm_d_ocp_qp_sol.h>
#include <hpipm_timing.h>
}
#include <tuple>

namespace ocs2
{

  struct MultipleShootingSolverSettings
  {
    size_t N;
    size_t nx;
    size_t nu;
    size_t sqpIteration;
    bool printSolverStatus;
    bool printSolverStatistics;
    bool printPrimalSol;
    bool initPrimalSol; // if false, use random matrix as init; if true, use the last PrimalSolution as init.
  };

  class MultipleShootingSolver : public Solver_BASE
  {
  public:
    /**
   * Pass dynamics costs etc. See what you need.
   */
    MultipleShootingSolver(MultipleShootingSolverSettings settings, const ocs2::SystemDynamicsBase *systemDynamicsPtr, const ocs2::CostFunctionBase *costFunctionPtr);

    ~MultipleShootingSolver() override = default;

    // TODO
    void reset() override;
    scalar_t getFinalTime() const override { return primalSolution_.timeTrajectory_.back(); }; // horizon is [t0, T] return T;
    // fill primal solution after solving the problem.
    void getPrimalSolution(scalar_t finalTime, PrimalSolution *primalSolutionPtr) const override
    {
      primalSolutionPtr->operator=(primalSolution_);
      if (settings_.printPrimalSol)
      {
        std::cout << "getting primal solution \n";
        for (int i = 0; i < settings_.N; i++)
        {
          std::cout << "time: " << primalSolutionPtr->timeTrajectory_[i]
                    << " state: " << primalSolutionPtr->stateTrajectory_[i].transpose()
                    << " input: " << primalSolutionPtr->inputTrajectory_[i].transpose()
                    << std::endl;
        }
      }
    }

    // Maybe Ignore
    const PerformanceIndex &getPerformanceIndeces() const override { return performanceIndex_; }
    size_t getNumIterations() const override { return 1; }
    const std::vector<PerformanceIndex> &getIterationsLog() const override { return performanceIndeces_; };
    const scalar_array_t &getPartitioningTimes() const override { return partitionTime_; };
    scalar_t getValueFunction(scalar_t time, const vector_t &state) const override { return 0.0; };
    void getValueFunctionStateDerivative(scalar_t time, const vector_t &state, vector_t &Vx) const override{};
    void getStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t &state, vector_t &nu) const override{};
    void rewindOptimizer(size_t firstIndex){};
    const unsigned long long int &getRewindCounter() const { 0; };

  private:
    void runImpl(scalar_t initTime, const vector_t &initState, scalar_t finalTime, const scalar_array_t &partitioningTimes) override;

    void runImpl(scalar_t initTime, const vector_t &initState, scalar_t finalTime, const scalar_array_t &partitioningTimes,
                 const std::vector<ControllerBase *> &controllersPtrStock) override { runImpl(initTime, initState, finalTime, partitioningTimes); }

    std::tuple<matrix_t, matrix_t> runSingleIter(SystemDynamicsBase &systemDynamicsPtr,
                                                 CostFunctionBase &costFunctionPtr,
                                                 scalar_t delta_t_,
                                                 scalar_t initTime,
                                                 const matrix_t &x,
                                                 const matrix_t &u,
                                                 const vector_t &initState);

    MultipleShootingSolverSettings settings_;
    std::unique_ptr<SystemDynamicsBase> systemDynamicsPtr_;
    std::unique_ptr<CostFunctionBase> costFunctionPtr_;

    PrimalSolution primalSolution_;

    // Unused : just to implement the interface
    PerformanceIndex performanceIndex_;
    std::vector<PerformanceIndex> performanceIndeces_ = {PerformanceIndex()};
    scalar_array_t partitionTime_;
  };

} // namespace ocs2