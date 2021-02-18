//
// Created by rgrandia on 09.11.20.
//

#pragma once
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_oc/oc_solver/SolverBase.h>
#include <iostream>

#include <ocs2_sqp/HpipmInterface.h>

#include <blasfeo_d_aux_ext_dep.h>
extern "C" {
#include <hpipm_d_ocp_qp.h>
#include <hpipm_d_ocp_qp_dim.h>
#include <hpipm_d_ocp_qp_ipm.h>
#include <hpipm_d_ocp_qp_sol.h>
#include <hpipm_timing.h>
}
#include <tuple>

namespace ocs2 {

struct MultipleShootingSolverSettings {
  size_t N;  // user-defined # of partition. finalTime - initTime = 1.0 seconds, so N = 10 -> delta_t = 0.1 s
  size_t n_state;
  size_t n_input;
  scalar_array_t trueEventTimes;  // size (N_real + 1), this is a modification to this->getModeSchedule().eventTimes. More info in function
                                  // getInfoFromModeSchedule.
  size_t N_real;                  // real # of partition, >= N, because there are some provided time instances must be covered
  size_t sqpIteration;
  scalar_t deltaTol;
  bool constrained;  // true for constrained systems, false for unconstrained systems
  bool qr_decomp;  // this variable is meaningful only if the system is constrained. True to use QR decomposiion, False to use lg <= Cx+Du+e
                   // <= ug
  bool printSolverStatus;
  bool printSolverStatistics;
  bool printPrimalSol;
  bool printModeScheduleDebug;
  bool initPrimalSol;  // if false, use random matrix as init; if true, use the last PrimalSolution as init. Internal use only.
  std::string robotName;
};

class MultipleShootingSolver : public SolverBase {
 public:
  /**
   * Pass dynamics costs etc. See what you need.
   */
  MultipleShootingSolver(MultipleShootingSolverSettings settings, const ocs2::SystemDynamicsBase* systemDynamicsPtr,
                         const ocs2::CostFunctionBase* costFunctionPtr, const ocs2::ConstraintBase* constraintPtr);

  ~MultipleShootingSolver() override = default;

  // TODO
  void reset() override;
  scalar_t getFinalTime() const override { return primalSolution_.timeTrajectory_.back(); };  // horizon is [t0, T] return T;
  // fill primal solution after solving the problem.
  void getPrimalSolution(scalar_t finalTime, PrimalSolution* primalSolutionPtr) const override {
    primalSolutionPtr->operator=(primalSolution_);
    if (settings_.printPrimalSol) {
      std::cout << "getting primal solution \n";
      for (int i = 0; i < settings_.N_real; i++) {
        std::cout << "time: " << primalSolutionPtr->timeTrajectory_[i] << " state: " << primalSolutionPtr->stateTrajectory_[i].transpose()
                  << " input: " << primalSolutionPtr->inputTrajectory_[i].transpose() << std::endl;
      }
    }
  }

  // self helper function
  void printPrimalSolution() {
    std::cout << "getting primal solution \n";
    for (int i = 0; i < settings_.N_real; i++) {
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
  void rewindOptimizer(size_t firstIndex){};
  const unsigned long long int& getRewindCounter() const { 0; };

 private:
  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes) override;

  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes,
               const std::vector<ControllerBase*>& controllersPtrStock) override {
    runImpl(initTime, initState, finalTime, partitioningTimes);
  }

  void setupCostDynamicsEqualityConstraint(SystemDynamicsBase& systemDynamicsObj, CostFunctionBase& costFunctionObj,
                                           ConstraintBase& constraintObj, const std::vector<ocs2::vector_t>& x, const std::vector<ocs2::vector_t>& u, const vector_t& initState);
  void setupDimension(ConstraintBase& constraintObj);
  std::pair<std::vector<ocs2::vector_t>, std::vector<ocs2::vector_t>> getOCPSolution(const vector_t& delta_x0);

  void getInfoFromModeSchedule(scalar_t initTime, scalar_t finalTime, ConstraintBase& constraintObj);

  MultipleShootingSolverSettings settings_;
  std::unique_ptr<SystemDynamicsBase> systemDynamicsPtr_;
  std::unique_ptr<CostFunctionBase> costFunctionPtr_;
  std::unique_ptr<ConstraintBase> constraintPtr_;

  PrimalSolution primalSolution_;

  HpipmInterface::OcpSize ocpSize;

  std::vector<VectorFunctionLinearApproximation> dynamics_;
  std::vector<ScalarFunctionQuadraticApproximation> cost_;

  // below are useful only when there are equality constraints that handled by QR decomposition
  matrix_array_t C_data;
  vector_array_t e_data;
  matrix_array_t Q1_data;
  matrix_array_t Q2_data;
  matrix_array_t R1_data;

  // below are useful when there are equality constraints ug >= C*dx + D*du >= lg, ug == lg
  // also useful for inequality constraints
  std::vector<VectorFunctionLinearApproximation> constraints_;

  // Unused : just to implement the interface
  PerformanceIndex performanceIndex_;
  std::vector<PerformanceIndex> performanceIndeces_ = {PerformanceIndex()};
  scalar_array_t partitionTime_;
};

}  // namespace ocs2