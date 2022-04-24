#pragma once

#include "ocs2_pipg/PIPG.h"

#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_core/integration/SensitivityIntegrator.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_core/thread_support/ThreadPool.h>

#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/oc_solver/SolverBase.h>

#include <ocs2_sqp/MultipleShootingSettings.h>
#include <ocs2_sqp/MultipleShootingSolverStatus.h>
#include <ocs2_sqp/TimeDiscretization.h>

namespace ocs2 {

class PipgMpcSolver : public SolverBase {
 public:
  /**
   * Constructor
   *
   * @param settings : settings for the multiple shooting solver.
   * @param [in] optimalControlProblem: The optimal control problem formulation.
   * @param [in] initializer: This class initializes the state-input for the time steps that no controller is available.
   */
  PipgMpcSolver(multiple_shooting::Settings sqpSettings, pipg::Settings pipgSettings, const OptimalControlProblem& optimalControlProblem,
                const Initializer& initializer);

  ~PipgMpcSolver() override;

  void reset() override;

  scalar_t getFinalTime() const override { return primalSolution_.timeTrajectory_.back(); };

  void getPrimalSolution(scalar_t finalTime, PrimalSolution* primalSolutionPtr) const override { *primalSolutionPtr = primalSolution_; }

  size_t getNumIterations() const override { return totalNumIterations_; }

  const PerformanceIndex& getPerformanceIndeces() const override { return getIterationsLog().back(); };

  const std::vector<PerformanceIndex>& getIterationsLog() const override;

  ScalarFunctionQuadraticApproximation getValueFunction(scalar_t time, const vector_t& state) const override {
    throw std::runtime_error("[PipgMpcSolver] getValueFunction() not available yet.");
  };

  ScalarFunctionQuadraticApproximation getHamiltonian(scalar_t time, const vector_t& state, const vector_t& input) override {
    throw std::runtime_error("[PipgMpcSolver] getHamiltonian() not available yet.");
  }

  vector_t getStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t& state) const override {
    throw std::runtime_error("[PipgMpcSolver] getStateInputEqualityConstraintLagrangian() not available yet.");
  }

 private:
  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime) override;

  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const ControllerBase* externalControllerPtr) override {
    if (externalControllerPtr == nullptr) {
      runImpl(initTime, initState, finalTime);
    } else {
      throw std::runtime_error("[PipgMpcSolver::run] This solver does not support external controller!");
    }
  }

  /** Run a task in parallel with settings.nThreads */
  void runParallel(std::function<void(int)> taskFunction);

  /** Get profiling information as a string */
  std::string getBenchmarkingInformation() const;

  std::string getBenchmarkingInformationPIPG() const;

  /** Initializes for the state-input trajectories */
  void initializeStateInputTrajectories(const vector_t& initState, const std::vector<AnnotatedTime>& timeDiscretization,
                                        vector_array_t& stateTrajectory, vector_array_t& inputTrajectory);

  /** Creates QP around t, x(t), u(t). Returns performance metrics at the current {t, x(t), u(t)} */
  PerformanceIndex setupQuadraticSubproblem(const std::vector<AnnotatedTime>& time, const vector_t& initState, const vector_array_t& x,
                                            const vector_array_t& u);

  /** Computes only the performance metrics at the current {t, x(t), u(t)} */
  PerformanceIndex computePerformance(const std::vector<AnnotatedTime>& time, const vector_t& initState, const vector_array_t& x,
                                      const vector_array_t& u);

  /** Returns solution of the QP subproblem in delta coordinates: */
  struct OcpSubproblemSolution {
    vector_array_t deltaXSol;      // delta_x(t)
    vector_array_t deltaUSol;      // delta_u(t)
    scalar_t armijoDescentMetric;  // inner product of the cost gradient and decision variable step
  };
  OcpSubproblemSolution getOCPSolution(const vector_t& delta_x0);

  /** Set up the primal solution based on the optimized state and input trajectories */
  void setPrimalSolution(const std::vector<AnnotatedTime>& time, vector_array_t&& x, vector_array_t&& u);

  /** Compute 2-norm of the trajectory: sqrt(sum_i v[i]^2)  */
  static scalar_t trajectoryNorm(const vector_array_t& v);

  /** Compute total constraint violation */
  scalar_t totalConstraintViolation(const PerformanceIndex& performance) const;

  /** Decides on the step to take and overrides given trajectories {x(t), u(t)} <- {x(t) + a*dx(t), u(t) + a*du(t)} */
  multiple_shooting::StepInfo takeStep(const PerformanceIndex& baseline, const std::vector<AnnotatedTime>& timeDiscretization,
                                       const vector_t& initState, const OcpSubproblemSolution& subproblemSolution, vector_array_t& x,
                                       vector_array_t& u);

  /** Determine convergence after a step */
  multiple_shooting::Convergence checkConvergence(int iteration, const PerformanceIndex& baseline,
                                                  const multiple_shooting::StepInfo& stepInfo) const;

  // Problem definition
  multiple_shooting::Settings settings_;
  DynamicsDiscretizer discretizer_;
  DynamicsSensitivityDiscretizer sensitivityDiscretizer_;
  std::vector<OptimalControlProblem> ocpDefinitions_;
  std::unique_ptr<Initializer> initializerPtr_;

  // Threading
  ThreadPool threadPool_;

  // Solution
  PrimalSolution primalSolution_;

  // Solver interface
  pipg::Settings pipgSettings_;
  Pipg pipgSolver_;

  // LQ approximation
  std::vector<VectorFunctionLinearApproximation> dynamics_;
  std::vector<ScalarFunctionQuadraticApproximation> cost_;
  std::vector<VectorFunctionLinearApproximation> constraints_;
  std::vector<VectorFunctionLinearApproximation> constraintsProjection_;

  // Iteration performance log
  std::vector<PerformanceIndex> performanceIndeces_;

  // Benchmarking
  size_t numProblems_{0};
  size_t totalNumIterations_{0};
  benchmark::RepeatedTimer initializationTimer_;
  benchmark::RepeatedTimer linearQuadraticApproximationTimer_;
  benchmark::RepeatedTimer solveQpTimer_;
  benchmark::RepeatedTimer linesearchTimer_;
  benchmark::RepeatedTimer computeControllerTimer_;

  // PIPG Solver
  benchmark::RepeatedTimer constructH_;
  benchmark::RepeatedTimer constructG_;
  benchmark::RepeatedTimer GTGMultiplication_;
  benchmark::RepeatedTimer lambdaEstimation_;
  benchmark::RepeatedTimer sigmaEstimation_;
  benchmark::RepeatedTimer preConditioning_;
};

}  // namespace ocs2
