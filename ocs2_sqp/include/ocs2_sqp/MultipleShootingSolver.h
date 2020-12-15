//
// Created by rgrandia on 09.11.20.
//

#pragma once
#include <iostream>
#include <ocs2_oc/oc_solver/SolverBase.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/constraint/ConstraintBase.h>

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
    size_t n_state;
    size_t n_input;
    size_t n_constraint;
    size_t sqpIteration;
    bool constrained; // true for constrained systems, false for unconstrained systems
    bool qr_decomp;   // this variable is meaningful only if the system is constrained. True to use QR decomposiion, false to use lg <= Cx+Du+e <= ug
    bool printSolverStatus;
    bool printSolverStatistics;
    bool printPrimalSol;
    bool initPrimalSol; // if false, use random matrix as init; if true, use the last PrimalSolution as init.
  };

  class MultipleShootingSolver : public SolverBase
  {
  public:
    /**
   * Pass dynamics costs etc. See what you need.
   */
    MultipleShootingSolver(MultipleShootingSolverSettings settings,
                           const ocs2::SystemDynamicsBase *systemDynamicsPtr,
                           const ocs2::CostFunctionBase *costFunctionPtr,
                           const ocs2::ConstraintBase *constraintPtr);

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

    // self helper function
    void printPrimalSolution()
    {
      std::cout << "getting primal solution \n";
      for (int i = 0; i < settings_.N; i++)
      {
        std::cout << "time: " << primalSolution_.timeTrajectory_[i]
                  << " state: " << primalSolution_.stateTrajectory_[i].transpose()
                  << " input: " << primalSolution_.inputTrajectory_[i].transpose()
                  << std::endl;
      }
    }

    // Maybe Ignore
    const PerformanceIndex &getPerformanceIndeces() const override { return performanceIndex_; }
    size_t getNumIterations() const override { return 1; }
    const std::vector<PerformanceIndex> &getIterationsLog() const override { return performanceIndeces_; };
    const scalar_array_t &getPartitioningTimes() const override { return partitionTime_; };
    scalar_t getValueFunction(scalar_t time, const vector_t &state) const override { return 0.0; };
    vector_t getValueFunctionStateDerivative(scalar_t time, const vector_t &state) const override{};
    void getStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t &state, vector_t &nu) const override{};
    void rewindOptimizer(size_t firstIndex){};
    const unsigned long long int &getRewindCounter() const { 0; };

  private:
    void runImpl(scalar_t initTime, const vector_t &initState, scalar_t finalTime, const scalar_array_t &partitioningTimes) override;

    void runImpl(scalar_t initTime, const vector_t &initState, scalar_t finalTime, const scalar_array_t &partitioningTimes,
                 const std::vector<ControllerBase *> &controllersPtrStock) override { runImpl(initTime, initState, finalTime, partitioningTimes); }

    void setupCostDynamicsEqualityConstraint(SystemDynamicsBase &systemDynamicsObj,
                                             CostFunctionBase &costFunctionObj,
                                             ConstraintBase &constraintObj,
                                             scalar_t delta_t_,
                                             scalar_t initTime,
                                             const matrix_t &x,
                                             const matrix_t &u,
                                             const vector_t &initState);
    void setupDimension();
    std::tuple<matrix_t, matrix_t> getOCPSolution(const vector_t &delta_x0);
    void solveOCP();
    void freeHPIPMMem();

    MultipleShootingSolverSettings settings_;
    std::unique_ptr<SystemDynamicsBase> systemDynamicsPtr_;
    std::unique_ptr<CostFunctionBase> costFunctionPtr_;
    std::unique_ptr<ConstraintBase> constraintPtr_;

    PrimalSolution primalSolution_;

    std::vector<int> nx_; //! number of states per stage
    std::vector<int> nu_; //! number of inputs per stage
    std::vector<int> ng_; //! number of general constraints per stage

    std::vector<int> nbu_; //! number of input box constraints per stage
    std::vector<int> nbx_; //! number of state box constraints per stage

    std::vector<int> nsbx_; // number of softed constraints on state box constraints
    std::vector<int> nsbu_; // number of softed constraints on input box constraints
    std::vector<int> nsg_;  // number of softed constraints on general constraints

    matrix_array_t A_data;
    matrix_array_t B_data;
    matrix_array_t b_data;
    matrix_array_t Q_data;
    matrix_array_t R_data;
    matrix_array_t S_data;
    matrix_array_t q_data;
    matrix_array_t r_data;

    // below are useful only when there are equality constraints that handled by QR decomposition
    matrix_array_t C_data;
    vector_array_t e_data;
    matrix_array_t Q1_data;
    matrix_array_t Q2_data;
    matrix_array_t R1_data;

    // below are useful when there are equality constraints ug >= C*dx + D*du >= lg, ug == lg
    // also useful for inequality constraints
    matrix_array_t C_C_data;
    matrix_array_t D_D_data;
    vector_array_t lg_data;
    vector_array_t ug_data;

    void *dim_mem;
    struct d_ocp_qp_dim dim;

    void *qp_mem;
    struct d_ocp_qp qp;

    void *qp_sol_mem;
    struct d_ocp_qp_sol qp_sol;

    void *ipm_arg_mem;
    struct d_ocp_qp_ipm_arg arg;

    // workspace
    void *ipm_mem;
    struct d_ocp_qp_ipm_ws workspace;
    int hpipm_status; // status code after solving

    // todo make this a setting
    ::hpipm_mode mode = ::hpipm_mode::SPEED; // ROBUST/BALANCED; see also hpipm_common.h

    // Unused : just to implement the interface
    PerformanceIndex performanceIndex_;
    std::vector<PerformanceIndex> performanceIndeces_ = {PerformanceIndex()};
    scalar_array_t partitionTime_;
  };

} // namespace ocs2