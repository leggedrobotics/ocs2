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

#include "ocs2_sqp/SqpSolver.h"

#include <iostream>
#include <numeric>

#include <ocs2_oc/multiple_shooting/Helpers.h>
#include <ocs2_oc/multiple_shooting/Initialization.h>
#include <ocs2_oc/multiple_shooting/PerformanceIndexComputation.h>
#include <ocs2_oc/multiple_shooting/Transcription.h>
#include <ocs2_oc/oc_problem/OcpSize.h>

namespace ocs2 {

namespace {
sqp::Settings rectifySettings(const OptimalControlProblem& ocp, sqp::Settings&& settings) {
  // True does not make sense if there are no constraints.
  if (ocp.equalityConstraintPtr->empty()) {
    settings.projectStateInputEqualityConstraints = false;
  }
  return settings;
}
}  // anonymous namespace

SqpSolver::SqpSolver(sqp::Settings settings, const OptimalControlProblem& optimalControlProblem, const Initializer& initializer)
    : settings_(rectifySettings(optimalControlProblem, std::move(settings))),
      hpipmInterface_(OcpSize(), settings_.hpipmSettings),
      threadPool_(std::max(settings_.nThreads, size_t(1)) - 1, settings_.threadPriority) {
  Eigen::setNbThreads(1);  // No multithreading within Eigen.
  Eigen::initParallel();

  // Dynamics discretization
  discretizer_ = selectDynamicsDiscretization(settings_.integratorType);
  sensitivityDiscretizer_ = selectDynamicsSensitivityDiscretization(settings_.integratorType);

  // Clone objects to have one for each worker
  for (int w = 0; w < settings_.nThreads; w++) {
    ocpDefinitions_.push_back(optimalControlProblem);
  }

  // Operating points
  initializerPtr_.reset(initializer.clone());

  // Linesearch
  filterLinesearch_.g_max = settings_.g_max;
  filterLinesearch_.g_min = settings_.g_min;
  filterLinesearch_.gamma_c = settings_.gamma_c;
  filterLinesearch_.armijoFactor = settings_.armijoFactor;
}

SqpSolver::~SqpSolver() {
  if (settings_.printSolverStatistics) {
    std::cerr << getBenchmarkingInformation() << std::endl;
  }
}

void SqpSolver::reset() {
  // Clear solution
  primalSolution_ = PrimalSolution();
  valueFunction_.clear();
  performanceIndeces_.clear();

  // reset timers
  numProblems_ = 0;
  totalNumIterations_ = 0;
  linearQuadraticApproximationTimer_.reset();
  solveQpTimer_.reset();
  linesearchTimer_.reset();
  computeControllerTimer_.reset();
}

std::string SqpSolver::getBenchmarkingInformation() const {
  const auto linearQuadraticApproximationTotal = linearQuadraticApproximationTimer_.getTotalInMilliseconds();
  const auto solveQpTotal = solveQpTimer_.getTotalInMilliseconds();
  const auto linesearchTotal = linesearchTimer_.getTotalInMilliseconds();
  const auto computeControllerTotal = computeControllerTimer_.getTotalInMilliseconds();

  const auto benchmarkTotal = linearQuadraticApproximationTotal + solveQpTotal + linesearchTotal + computeControllerTotal;

  std::stringstream infoStream;
  if (benchmarkTotal > 0.0) {
    const scalar_t inPercent = 100.0;
    infoStream << "\n########################################################################\n";
    infoStream << "The benchmarking is computed over " << totalNumIterations_ << " iterations. \n";
    infoStream << "SQP Benchmarking\t   :\tAverage time [ms]   (% of total runtime)\n";
    infoStream << "\tLQ Approximation   :\t" << linearQuadraticApproximationTimer_.getAverageInMilliseconds() << " [ms] \t\t("
               << linearQuadraticApproximationTotal / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tSolve QP           :\t" << solveQpTimer_.getAverageInMilliseconds() << " [ms] \t\t("
               << solveQpTotal / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tLinesearch         :\t" << linesearchTimer_.getAverageInMilliseconds() << " [ms] \t\t("
               << linesearchTotal / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tCompute Controller :\t" << computeControllerTimer_.getAverageInMilliseconds() << " [ms] \t\t("
               << computeControllerTotal / benchmarkTotal * inPercent << "%)\n";
  }
  return infoStream.str();
}

const std::vector<PerformanceIndex>& SqpSolver::getIterationsLog() const {
  if (performanceIndeces_.empty()) {
    throw std::runtime_error("[SqpSolver]: No performance log yet, no problem solved yet?");
  } else {
    return performanceIndeces_;
  }
}

ScalarFunctionQuadraticApproximation SqpSolver::getValueFunction(scalar_t time, const vector_t& state) const {
  if (valueFunction_.empty()) {
    throw std::runtime_error("[SqpSolver] Value function is empty! Is createValueFunction true and did the solver run?");
  } else {
    // Interpolation
    const auto indexAlpha = LinearInterpolation::timeSegment(time, primalSolution_.timeTrajectory_);

    ScalarFunctionQuadraticApproximation valueFunction;
    using T = std::vector<ocs2::ScalarFunctionQuadraticApproximation>;
    using LinearInterpolation::interpolate;
    valueFunction.f = 0.0;
    valueFunction.dfdx = interpolate(indexAlpha, valueFunction_, [](const T& v, size_t ind) -> const vector_t& { return v[ind].dfdx; });
    valueFunction.dfdxx = interpolate(indexAlpha, valueFunction_, [](const T& v, size_t ind) -> const matrix_t& { return v[ind].dfdxx; });

    // Re-center around query state
    valueFunction.dfdx.noalias() += valueFunction.dfdxx * state;

    return valueFunction;
  }
}

void SqpSolver::runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime) {
  if (settings_.printSolverStatus || settings_.printLinesearch) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n+++++++++++++ SQP solver is initialized ++++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
  }

  // Determine time discretization, taking into account event times.
  const auto& eventTimes = this->getReferenceManager().getModeSchedule().eventTimes;
  const auto timeDiscretization = timeDiscretizationWithEvents(initTime, finalTime, settings_.dt, eventTimes);

  // Initialize references
  for (auto& ocpDefinition : ocpDefinitions_) {
    const auto& targetTrajectories = this->getReferenceManager().getTargetTrajectories();
    ocpDefinition.targetTrajectoriesPtr = &targetTrajectories;
  }

  // Initialize the state and input
  vector_array_t x, u;
  multiple_shooting::initializeStateInputTrajectories(initState, timeDiscretization, primalSolution_, *initializerPtr_, x, u);

  // Bookkeeping
  performanceIndeces_.clear();

  int iter = 0;
  sqp::Convergence convergence = sqp::Convergence::FALSE;
  while (convergence == sqp::Convergence::FALSE) {
    if (settings_.printSolverStatus || settings_.printLinesearch) {
      std::cerr << "\nSQP iteration: " << iter << "\n";
    }
    // Make QP approximation
    linearQuadraticApproximationTimer_.startTimer();
    const auto baselinePerformance = setupQuadraticSubproblem(timeDiscretization, initState, x, u);
    linearQuadraticApproximationTimer_.endTimer();

    // Solve QP
    solveQpTimer_.startTimer();
    const vector_t delta_x0 = initState - x[0];
    const auto deltaSolution = getOCPSolution(delta_x0);
    extractValueFunction(timeDiscretization, x);
    solveQpTimer_.endTimer();

    // Apply step
    linesearchTimer_.startTimer();
    const auto stepInfo = takeStep(baselinePerformance, timeDiscretization, initState, deltaSolution, x, u);
    performanceIndeces_.push_back(stepInfo.performanceAfterStep);
    linesearchTimer_.endTimer();

    // Check convergence
    convergence = checkConvergence(iter, baselinePerformance, stepInfo);

    // Next iteration
    ++iter;
    ++totalNumIterations_;
  }

  computeControllerTimer_.startTimer();
  primalSolution_ = toPrimalSolution(timeDiscretization, std::move(x), std::move(u));
  computeControllerTimer_.endTimer();

  ++numProblems_;

  if (settings_.printSolverStatus || settings_.printLinesearch) {
    std::cerr << "\nConvergence : " << toString(convergence) << "\n";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n+++++++++++++ SQP solver has terminated ++++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
  }
}

void SqpSolver::runParallel(std::function<void(int)> taskFunction) {
  threadPool_.runParallel(std::move(taskFunction), settings_.nThreads);
}

SqpSolver::OcpSubproblemSolution SqpSolver::getOCPSolution(const vector_t& delta_x0) {
  // Solve the QP
  OcpSubproblemSolution solution;
  auto& deltaXSol = solution.deltaXSol;
  auto& deltaUSol = solution.deltaUSol;
  hpipm_status status;
  const bool hasStateInputConstraints = !ocpDefinitions_.front().equalityConstraintPtr->empty();
  if (hasStateInputConstraints && !settings_.projectStateInputEqualityConstraints) {
    hpipmInterface_.resize(extractSizesFromProblem(dynamics_, cost_, &stateInputEqConstraints_));
    status =
        hpipmInterface_.solve(delta_x0, dynamics_, cost_, &stateInputEqConstraints_, deltaXSol, deltaUSol, settings_.printSolverStatus);
  } else {  // without constraints, or when using projection, we have an unconstrained QP.
    hpipmInterface_.resize(extractSizesFromProblem(dynamics_, cost_, nullptr));
    status = hpipmInterface_.solve(delta_x0, dynamics_, cost_, nullptr, deltaXSol, deltaUSol, settings_.printSolverStatus);
  }

  if (status != hpipm_status::SUCCESS) {
    throw std::runtime_error("[SqpSolver] Failed to solve QP");
  }

  // to determine if the solution is a descent direction for the cost: compute gradient(cost)' * [dx; du]
  solution.armijoDescentMetric = armijoDescentMetric(cost_, deltaXSol, deltaUSol);

  // remap the tilde delta u to real delta u
  if (settings_.projectStateInputEqualityConstraints) {
    multiple_shooting::remapProjectedInput(constraintsProjection_, deltaXSol, deltaUSol);
  }

  return solution;
}

void SqpSolver::extractValueFunction(const std::vector<AnnotatedTime>& time, const vector_array_t& x) {
  if (settings_.createValueFunction) {
    valueFunction_ = hpipmInterface_.getRiccatiCostToGo(dynamics_[0], cost_[0]);
    // Correct for linearization state
    for (int i = 0; i < time.size(); ++i) {
      valueFunction_[i].dfdx.noalias() -= valueFunction_[i].dfdxx * x[i];
    }
  }
}

PrimalSolution SqpSolver::toPrimalSolution(const std::vector<AnnotatedTime>& time, vector_array_t&& x, vector_array_t&& u) {
  if (settings_.useFeedbackPolicy) {
    ModeSchedule modeSchedule = this->getReferenceManager().getModeSchedule();
    matrix_array_t KMatrices = hpipmInterface_.getRiccatiFeedback(dynamics_[0], cost_[0]);
    if (settings_.projectStateInputEqualityConstraints) {
      multiple_shooting::remapProjectedGain(constraintsProjection_, KMatrices);
    }
    return multiple_shooting::toPrimalSolution(time, std::move(modeSchedule), std::move(x), std::move(u), std::move(KMatrices));

  } else {
    ModeSchedule modeSchedule = this->getReferenceManager().getModeSchedule();
    return multiple_shooting::toPrimalSolution(time, std::move(modeSchedule), std::move(x), std::move(u));
  }
}

PerformanceIndex SqpSolver::setupQuadraticSubproblem(const std::vector<AnnotatedTime>& time, const vector_t& initState,
                                                     const vector_array_t& x, const vector_array_t& u) {
  // Problem horizon
  const int N = static_cast<int>(time.size()) - 1;

  std::vector<PerformanceIndex> performance(settings_.nThreads, PerformanceIndex());
  cost_.resize(N + 1);
  dynamics_.resize(N);
  stateInputEqConstraints_.resize(N);
  stateIneqConstraints_.resize(N + 1);
  stateInputIneqConstraints_.resize(N);
  constraintsProjection_.resize(N);
  projectionMultiplierCoefficients_.resize(N);

  std::atomic_int timeIndex{0};
  auto parallelTask = [&](int workerId) {
    // Get worker specific resources
    OptimalControlProblem& ocpDefinition = ocpDefinitions_[workerId];
    PerformanceIndex workerPerformance;  // Accumulate performance in local variable

    int i = timeIndex++;
    while (i < N) {
      if (time[i].event == AnnotatedTime::Event::PreEvent) {
        // Event node
        auto result = multiple_shooting::setupEventNode(ocpDefinition, time[i].time, x[i], x[i + 1]);
        workerPerformance += multiple_shooting::computeEventPerformance(result);
        cost_[i] = std::move(result.cost);
        dynamics_[i] = std::move(result.dynamics);
        stateInputEqConstraints_[i] = VectorFunctionLinearApproximation(0, x[i].size(), 0);
        stateIneqConstraints_[i] = std::move(result.ineqConstraints);
        stateInputIneqConstraints_[i] = VectorFunctionLinearApproximation::Zero(0, x[i].size(), 0);
        constraintsProjection_[i] = VectorFunctionLinearApproximation::Zero(0, x[i].size(), 0);
        projectionMultiplierCoefficients_[i] = multiple_shooting::ProjectionMultiplierCoefficients();
      } else {
        // Normal, intermediate node
        const scalar_t ti = getIntervalStart(time[i]);
        const scalar_t dt = getIntervalDuration(time[i], time[i + 1]);
        auto result = multiple_shooting::setupIntermediateNode(ocpDefinition, sensitivityDiscretizer_, ti, dt, x[i], x[i + 1], u[i]);
        workerPerformance += multiple_shooting::computeIntermediatePerformance(result, dt);
        if (settings_.projectStateInputEqualityConstraints) {
          multiple_shooting::projectTranscription(result, settings_.extractProjectionMultiplier);
        }
        cost_[i] = std::move(result.cost);
        dynamics_[i] = std::move(result.dynamics);
        stateInputEqConstraints_[i] = std::move(result.stateInputEqConstraints);
        stateIneqConstraints_[i] = std::move(result.stateIneqConstraints);
        stateInputIneqConstraints_[i] = std::move(result.stateInputIneqConstraints);
        constraintsProjection_[i] = std::move(result.constraintsProjection);
        projectionMultiplierCoefficients_[i] = std::move(result.projectionMultiplierCoefficients);
      }

      i = timeIndex++;
    }

    if (i == N) {  // Only one worker will execute this
      const scalar_t tN = getIntervalStart(time[N]);
      auto result = multiple_shooting::setupTerminalNode(ocpDefinition, tN, x[N]);
      workerPerformance += multiple_shooting::computeTerminalPerformance(result);
      cost_[i] = std::move(result.cost);
      stateIneqConstraints_[i] = std::move(result.ineqConstraints);
    }

    // Accumulate! Same worker might run multiple tasks
    performance[workerId] += workerPerformance;
  };
  runParallel(std::move(parallelTask));

  // Account for init state in performance
  performance.front().dynamicsViolationSSE += (initState - x.front()).squaredNorm();

  // Sum performance of the threads
  PerformanceIndex totalPerformance = std::accumulate(std::next(performance.begin()), performance.end(), performance.front());
  totalPerformance.merit = totalPerformance.cost + totalPerformance.equalityLagrangian + totalPerformance.inequalityLagrangian;

  return totalPerformance;
}

PerformanceIndex SqpSolver::computePerformance(const std::vector<AnnotatedTime>& time, const vector_t& initState, const vector_array_t& x,
                                               const vector_array_t& u) {
  // Problem horizon
  const int N = static_cast<int>(time.size()) - 1;

  std::vector<PerformanceIndex> performance(settings_.nThreads, PerformanceIndex());
  std::atomic_int timeIndex{0};
  auto parallelTask = [&](int workerId) {
    // Get worker specific resources
    OptimalControlProblem& ocpDefinition = ocpDefinitions_[workerId];
    PerformanceIndex workerPerformance;  // Accumulate performance in local variable

    int i = timeIndex++;
    while (i < N) {
      if (time[i].event == AnnotatedTime::Event::PreEvent) {
        // Event node
        workerPerformance += multiple_shooting::computeEventPerformance(ocpDefinition, time[i].time, x[i], x[i + 1]);
      } else {
        // Normal, intermediate node
        const scalar_t ti = getIntervalStart(time[i]);
        const scalar_t dt = getIntervalDuration(time[i], time[i + 1]);
        workerPerformance += multiple_shooting::computeIntermediatePerformance(ocpDefinition, discretizer_, ti, dt, x[i], x[i + 1], u[i]);
      }

      i = timeIndex++;
    }

    if (i == N) {  // Only one worker will execute this
      const scalar_t tN = getIntervalStart(time[N]);
      workerPerformance += multiple_shooting::computeTerminalPerformance(ocpDefinition, tN, x[N]);
    }

    // Accumulate! Same worker might run multiple tasks
    performance[workerId] += workerPerformance;
  };
  runParallel(std::move(parallelTask));

  // Account for init state in performance
  performance.front().dynamicsViolationSSE += (initState - x.front()).squaredNorm();

  // Sum performance of the threads
  PerformanceIndex totalPerformance = std::accumulate(std::next(performance.begin()), performance.end(), performance.front());
  totalPerformance.merit = totalPerformance.cost + totalPerformance.equalityLagrangian + totalPerformance.inequalityLagrangian;
  return totalPerformance;
}

sqp::StepInfo SqpSolver::takeStep(const PerformanceIndex& baseline, const std::vector<AnnotatedTime>& timeDiscretization,
                                  const vector_t& initState, const OcpSubproblemSolution& subproblemSolution, vector_array_t& x,
                                  vector_array_t& u) {
  using StepType = FilterLinesearch::StepType;

  /*
   * Filter linesearch based on:
   * "On the implementation of an interior-point filter line-search algorithm for large-scale nonlinear programming"
   * https://link.springer.com/article/10.1007/s10107-004-0559-y
   */
  if (settings_.printLinesearch) {
    std::cerr << std::setprecision(9) << std::fixed;
    std::cerr << "\n=== Linesearch ===\n";
    std::cerr << "Baseline:\n" << baseline << "\n";
  }

  // Baseline costs
  const scalar_t baselineConstraintViolation = FilterLinesearch::totalConstraintViolation(baseline);

  // Update norm
  const auto& dx = subproblemSolution.deltaXSol;
  const auto& du = subproblemSolution.deltaUSol;
  const auto deltaUnorm = multiple_shooting::trajectoryNorm(du);
  const auto deltaXnorm = multiple_shooting::trajectoryNorm(dx);

  scalar_t alpha = 1.0;
  vector_array_t xNew(x.size());
  vector_array_t uNew(u.size());
  do {
    // Compute step
    multiple_shooting::incrementTrajectory(u, du, alpha, uNew);
    multiple_shooting::incrementTrajectory(x, dx, alpha, xNew);

    // Compute cost and constraints
    const PerformanceIndex performanceNew = computePerformance(timeDiscretization, initState, xNew, uNew);

    // Step acceptance and record step type
    bool stepAccepted;
    StepType stepType;
    std::tie(stepAccepted, stepType) =
        filterLinesearch_.acceptStep(baseline, performanceNew, alpha * subproblemSolution.armijoDescentMetric);

    if (settings_.printLinesearch) {
      std::cerr << "Step size: " << alpha << ", Step Type: " << toString(stepType)
                << (stepAccepted ? std::string{" (Accepted)"} : std::string{" (Rejected)"}) << "\n";
      std::cerr << "|dx| = " << alpha * deltaXnorm << "\t|du| = " << alpha * deltaUnorm << "\n";
      std::cerr << performanceNew << "\n";
    }

    if (stepAccepted) {  // Return if step accepted
      x = std::move(xNew);
      u = std::move(uNew);

      // Prepare step info
      sqp::StepInfo stepInfo;
      stepInfo.stepSize = alpha;
      stepInfo.stepType = stepType;
      stepInfo.dx_norm = alpha * deltaXnorm;
      stepInfo.du_norm = alpha * deltaUnorm;
      stepInfo.performanceAfterStep = performanceNew;
      stepInfo.totalConstraintViolationAfterStep = FilterLinesearch::totalConstraintViolation(performanceNew);
      return stepInfo;

    } else {  // Try smaller step
      alpha *= settings_.alpha_decay;

      // Detect too small step size during back-tracking to escape early. Prevents going all the way to alpha_min
      if (alpha * deltaXnorm < settings_.deltaTol && alpha * deltaUnorm < settings_.deltaTol) {
        if (settings_.printLinesearch) {
          std::cerr << "Exiting linesearch early due to too small primal steps |dx|: " << alpha * deltaXnorm
                    << ", and or |du|: " << alpha * deltaUnorm << " are below deltaTol: " << settings_.deltaTol << "\n";
        }
        break;
      }
    }
  } while (alpha >= settings_.alpha_min);

  // Alpha_min reached -> Don't take a step
  sqp::StepInfo stepInfo;
  stepInfo.stepSize = 0.0;
  stepInfo.stepType = StepType::ZERO;
  stepInfo.dx_norm = 0.0;
  stepInfo.du_norm = 0.0;
  stepInfo.performanceAfterStep = baseline;
  stepInfo.totalConstraintViolationAfterStep = FilterLinesearch::totalConstraintViolation(baseline);

  if (settings_.printLinesearch) {
    std::cerr << "[Linesearch terminated] Step size: " << stepInfo.stepSize << ", Step Type: " << toString(stepInfo.stepType) << "\n";
  }

  return stepInfo;
}

sqp::Convergence SqpSolver::checkConvergence(int iteration, const PerformanceIndex& baseline, const sqp::StepInfo& stepInfo) const {
  using Convergence = sqp::Convergence;
  if ((iteration + 1) >= settings_.sqpIteration) {
    // Converged because the next iteration would exceed the specified number of iterations
    return Convergence::ITERATIONS;
  } else if (stepInfo.stepSize < settings_.alpha_min) {
    // Converged because step size is below the specified minimum
    return Convergence::STEPSIZE;
  } else if (std::abs(stepInfo.performanceAfterStep.merit - baseline.merit) < settings_.costTol &&
             FilterLinesearch::totalConstraintViolation(stepInfo.performanceAfterStep) < settings_.g_min) {
    // Converged because the change in merit is below the specified tolerance while the constraint violation is below the minimum
    return Convergence::METRICS;
  } else if (stepInfo.dx_norm < settings_.deltaTol && stepInfo.du_norm < settings_.deltaTol) {
    // Converged because the change in primal variables is below the specified tolerance
    return Convergence::PRIMAL;
  } else {
    // None of the above convergence criteria were met -> not converged.
    return Convergence::FALSE;
  }
}

}  // namespace ocs2
