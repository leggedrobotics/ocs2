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

#include "ocs2_slp/SlpSolver.h"

#include <iomanip>
#include <iostream>
#include <numeric>

#include <ocs2_oc/multiple_shooting/Helpers.h>
#include <ocs2_oc/multiple_shooting/Initialization.h>
#include <ocs2_oc/multiple_shooting/MetricsComputation.h>
#include <ocs2_oc/multiple_shooting/PerformanceIndexComputation.h>
#include <ocs2_oc/multiple_shooting/Transcription.h>
#include <ocs2_oc/precondition/Ruzi.h>
#include <ocs2_oc/trajectory_adjustment/TrajectorySpreadingHelperFunctions.h>

#include "ocs2_slp/Helpers.h"

namespace ocs2 {

SlpSolver::SlpSolver(slp::Settings settings, const OptimalControlProblem& optimalControlProblem, const Initializer& initializer)
    : settings_(std::move(settings)),
      pipgSolver_(settings_.pipgSettings),
      threadPool_(std::max(settings_.nThreads - 1, size_t(1)) - 1, settings_.threadPriority) {
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

SlpSolver::~SlpSolver() {
  if (settings_.printSolverStatistics) {
    std::cerr << getBenchmarkingInformationPIPG() << "\n" << getBenchmarkingInformation() << std::endl;
  }
}

void SlpSolver::reset() {
  // Clear solution
  primalSolution_ = PrimalSolution();
  performanceIndeces_.clear();

  // reset timers
  numProblems_ = 0;
  totalNumIterations_ = 0;
  initializationTimer_.reset();
  linearQuadraticApproximationTimer_.reset();
  solveQpTimer_.reset();
  linesearchTimer_.reset();
  computeControllerTimer_.reset();
  lambdaEstimation_.reset();
  sigmaEstimation_.reset();
  preConditioning_.reset();
  pipgSolverTimer_.reset();
}

std::string SlpSolver::getBenchmarkingInformationPIPG() const {
  const auto lambdaEstimation = lambdaEstimation_.getTotalInMilliseconds();
  const auto sigmaEstimation = sigmaEstimation_.getTotalInMilliseconds();
  const auto preConditioning = preConditioning_.getTotalInMilliseconds();
  const auto pipgRuntime = pipgSolverTimer_.getTotalInMilliseconds();

  const auto benchmarkTotal = preConditioning + lambdaEstimation + sigmaEstimation + pipgRuntime;

  std::stringstream infoStream;
  if (benchmarkTotal > 0.0) {
    const scalar_t inPercent = 100.0;
    infoStream << "\n########################################################################\n";
    infoStream << "The benchmarking is computed over " << preConditioning_.getNumTimedIntervals() << " iterations. \n";
    infoStream << "PIPG Benchmarking\t       :\tAverage time [ms]   (% of total runtime)\n";
    infoStream << "\tpreConditioning        :\t" << std::setw(10) << preConditioning_.getAverageInMilliseconds() << " [ms] \t("
               << preConditioning / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tlambdaEstimation       :\t" << std::setw(10) << lambdaEstimation_.getAverageInMilliseconds() << " [ms] \t("
               << lambdaEstimation / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tsigmaEstimation        :\t" << std::setw(10) << sigmaEstimation_.getAverageInMilliseconds() << " [ms] \t("
               << sigmaEstimation / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tPIPG runTime           :\t" << std::setw(10) << pipgSolverTimer_.getAverageInMilliseconds() << " [ms] \t("
               << pipgRuntime / benchmarkTotal * inPercent << "%)\n";
  }
  return infoStream.str();
}

std::string SlpSolver::getBenchmarkingInformation() const {
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
    infoStream << "SLP Benchmarking\t   :\tAverage time [ms]   (% of total runtime)\n";
    infoStream << "\tLQ Approximation   :\t" << std::setw(10) << linearQuadraticApproximationTimer_.getAverageInMilliseconds()
               << " [ms] \t(" << linearQuadraticApproximationTotal / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tSolve LP           :\t" << std::setw(10) << solveQpTimer_.getAverageInMilliseconds() << " [ms] \t("
               << solveQpTotal / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tLinesearch         :\t" << std::setw(10) << linesearchTimer_.getAverageInMilliseconds() << " [ms] \t("
               << linesearchTotal / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tCompute Controller :\t" << std::setw(10) << computeControllerTimer_.getAverageInMilliseconds() << " [ms] \t("
               << computeControllerTotal / benchmarkTotal * inPercent << "%)\n";
  }
  return infoStream.str();
}

const std::vector<PerformanceIndex>& SlpSolver::getIterationsLog() const {
  if (performanceIndeces_.empty()) {
    throw std::runtime_error("[SlpSolver]: No performance log yet, no problem solved yet?");
  } else {
    return performanceIndeces_;
  }
}

void SlpSolver::runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime) {
  if (settings_.printSolverStatus || settings_.printLinesearch) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n+++++++++++++ SLP solver is initialized ++++++++++++++";
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

  // Trajectory spread of primalSolution_
  if (!primalSolution_.timeTrajectory_.empty()) {
    std::ignore = trajectorySpread(primalSolution_.modeSchedule_, this->getReferenceManager().getModeSchedule(), primalSolution_);
  }

  // Initialize the state and input
  vector_array_t x, u;
  multiple_shooting::initializeStateInputTrajectories(initState, timeDiscretization, primalSolution_, *initializerPtr_, x, u);

  // Bookkeeping
  performanceIndeces_.clear();
  std::vector<Metrics> metrics;

  int iter = 0;
  slp::Convergence convergence = slp::Convergence::FALSE;
  while (convergence == slp::Convergence::FALSE) {
    if (settings_.printSolverStatus || settings_.printLinesearch) {
      std::cerr << "\nPIPG iteration: " << iter << "\n";
    }
    // Make QP approximation
    linearQuadraticApproximationTimer_.startTimer();
    const auto baselinePerformance = setupQuadraticSubproblem(timeDiscretization, initState, x, u, metrics);
    linearQuadraticApproximationTimer_.endTimer();

    // Solve LP
    solveQpTimer_.startTimer();
    const vector_t delta_x0 = initState - x[0];
    const auto deltaSolution = getOCPSolution(delta_x0);
    solveQpTimer_.endTimer();

    // Apply step
    linesearchTimer_.startTimer();
    const auto stepInfo = takeStep(baselinePerformance, timeDiscretization, initState, deltaSolution, x, u, metrics);
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
  problemMetrics_ = multiple_shooting::toProblemMetrics(timeDiscretization, std::move(metrics));
  computeControllerTimer_.endTimer();

  ++numProblems_;

  if (settings_.printSolverStatus || settings_.printLinesearch) {
    std::cerr << "\nConvergence : " << toString(convergence) << "\n";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n+++++++++++++ SLP solver has terminated ++++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
  }
}

void SlpSolver::runParallel(std::function<void(int)> taskFunction) {
  threadPool_.runParallel(std::move(taskFunction), settings_.nThreads);
}

SlpSolver::OcpSubproblemSolution SlpSolver::getOCPSolution(const vector_t& delta_x0) {
  // Solve the QP
  OcpSubproblemSolution solution;
  auto& deltaXSol = solution.deltaXSol;
  auto& deltaUSol = solution.deltaUSol;

  // without constraints, or when using projection, we have an unconstrained QP.
  pipgSolver_.resize(extractSizesFromProblem(dynamics_, cost_, nullptr));

  // pre-condition the OCP
  preConditioning_.startTimer();
  scalar_t c;
  vector_array_t D, E;
  vector_array_t scalingVectors;
  precondition::ocpDataInPlaceInParallel(threadPool_, delta_x0, pipgSolver_.size(), settings_.scalingIteration, dynamics_, cost_, D, E,
                                         scalingVectors, c);
  preConditioning_.endTimer();

  // estimate mu and lambda: mu I < H < lambda I
  const auto muEstimated = [&]() {
    scalar_t maxScalingFactor = -1;
    for (auto& v : D) {
      if (v.size() != 0) {
        maxScalingFactor = std::max(maxScalingFactor, v.maxCoeff());
      }
    }
    return c * pipgSolver_.settings().lowerBoundH * maxScalingFactor * maxScalingFactor;
  }();
  lambdaEstimation_.startTimer();
  const auto lambdaScaled = slp::hessianEigenvaluesUpperBound(pipgSolver_.size(), cost_);
  lambdaEstimation_.endTimer();

  // estimate sigma: G' G < sigma I
  // However, since the G'G and GG' have exactly the same set of eigenvalues value: G G' < sigma I
  sigmaEstimation_.startTimer();
  const auto sigmaScaled = slp::GGTEigenvaluesUpperBound(threadPool_, pipgSolver_.size(), dynamics_, nullptr, &scalingVectors);
  sigmaEstimation_.endTimer();

  pipgSolverTimer_.startTimer();
  vector_array_t EInv(E.size());
  std::transform(E.begin(), E.end(), EInv.begin(), [](const vector_t& v) { return v.cwiseInverse(); });
  const pipg::PipgBounds pipgBounds{muEstimated, lambdaScaled, sigmaScaled};
  const auto pipgStatus =
      pipgSolver_.solve(threadPool_, delta_x0, dynamics_, cost_, nullptr, scalingVectors, &EInv, pipgBounds, deltaXSol, deltaUSol);
  pipgSolverTimer_.endTimer();

  // to determine if the solution is a descent direction for the cost: compute gradient(cost)' * [dx; du]
  solution.armijoDescentMetric = armijoDescentMetric(cost_, deltaXSol, deltaUSol);

  precondition::descaleSolution(D, deltaXSol, deltaUSol);

  // remap the tilde delta u to real delta u
  multiple_shooting::remapProjectedInput(constraintsProjection_, deltaXSol, deltaUSol);

  return solution;
}

PrimalSolution SlpSolver::toPrimalSolution(const std::vector<AnnotatedTime>& time, vector_array_t&& x, vector_array_t&& u) {
  ModeSchedule modeSchedule = this->getReferenceManager().getModeSchedule();
  return multiple_shooting::toPrimalSolution(time, std::move(modeSchedule), std::move(x), std::move(u));
}

PerformanceIndex SlpSolver::setupQuadraticSubproblem(const std::vector<AnnotatedTime>& time, const vector_t& initState,
                                                     const vector_array_t& x, const vector_array_t& u, std::vector<Metrics>& metrics) {
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
  metrics.resize(N + 1);

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
        metrics[i] = multiple_shooting::computeMetrics(result);
        workerPerformance += multiple_shooting::computePerformanceIndex(result);
        cost_[i] = std::move(result.cost);
        dynamics_[i] = std::move(result.dynamics);
        stateInputEqConstraints_[i].resize(0, x[i].size());
        stateIneqConstraints_[i] = std::move(result.ineqConstraints);
        stateInputIneqConstraints_[i].resize(0, x[i].size());
        constraintsProjection_[i].resize(0, x[i].size());
        projectionMultiplierCoefficients_[i] = multiple_shooting::ProjectionMultiplierCoefficients();
      } else {
        // Normal, intermediate node
        const scalar_t ti = getIntervalStart(time[i]);
        const scalar_t dt = getIntervalDuration(time[i], time[i + 1]);
        auto result = multiple_shooting::setupIntermediateNode(ocpDefinition, sensitivityDiscretizer_, ti, dt, x[i], x[i + 1], u[i]);
        metrics[i] = multiple_shooting::computeMetrics(result);
        workerPerformance += multiple_shooting::computePerformanceIndex(result, dt);
        multiple_shooting::projectTranscription(result, settings_.extractProjectionMultiplier);
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
      metrics[i] = multiple_shooting::computeMetrics(result);
      workerPerformance += multiple_shooting::computePerformanceIndex(result);
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

PerformanceIndex SlpSolver::computePerformance(const std::vector<AnnotatedTime>& time, const vector_t& initState, const vector_array_t& x,
                                               const vector_array_t& u, std::vector<Metrics>& metrics) {
  // Problem size
  const int N = static_cast<int>(time.size()) - 1;
  metrics.resize(N + 1);

  std::vector<PerformanceIndex> performance(settings_.nThreads, PerformanceIndex());
  std::atomic_int timeIndex{0};
  auto parallelTask = [&](int workerId) {
    // Get worker specific resources
    OptimalControlProblem& ocpDefinition = ocpDefinitions_[workerId];

    int i = timeIndex++;
    while (i < N) {
      if (time[i].event == AnnotatedTime::Event::PreEvent) {
        // Event node
        metrics[i] = multiple_shooting::computeEventMetrics(ocpDefinition, time[i].time, x[i], x[i + 1]);
        performance[workerId] += toPerformanceIndex(metrics[i]);
      } else {
        // Normal, intermediate node
        const scalar_t ti = getIntervalStart(time[i]);
        const scalar_t dt = getIntervalDuration(time[i], time[i + 1]);
        metrics[i] = multiple_shooting::computeIntermediateMetrics(ocpDefinition, discretizer_, ti, dt, x[i], x[i + 1], u[i]);
        performance[workerId] += toPerformanceIndex(metrics[i], dt);
      }

      i = timeIndex++;
    }

    if (i == N) {  // Only one worker will execute this
      const scalar_t tN = getIntervalStart(time[N]);
      metrics[N] = multiple_shooting::computeTerminalMetrics(ocpDefinition, tN, x[N]);
      performance[workerId] += toPerformanceIndex(metrics[N]);
    }
  };
  runParallel(std::move(parallelTask));

  // Account for initial state in performance
  const vector_t initDynamicsViolation = initState - x.front();
  metrics.front().dynamicsViolation += initDynamicsViolation;
  performance.front().dynamicsViolationSSE += initDynamicsViolation.squaredNorm();

  // Sum performance of the threads
  PerformanceIndex totalPerformance = std::accumulate(std::next(performance.begin()), performance.end(), performance.front());
  totalPerformance.merit = totalPerformance.cost + totalPerformance.equalityLagrangian + totalPerformance.inequalityLagrangian;
  return totalPerformance;
}

slp::StepInfo SlpSolver::takeStep(const PerformanceIndex& baseline, const std::vector<AnnotatedTime>& timeDiscretization,
                                  const vector_t& initState, const OcpSubproblemSolution& subproblemSolution, vector_array_t& x,
                                  vector_array_t& u, std::vector<Metrics>& metrics) {
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
  std::vector<Metrics> metricsNew(metrics.size());
  do {
    // Compute step
    multiple_shooting::incrementTrajectory(u, du, alpha, uNew);
    multiple_shooting::incrementTrajectory(x, dx, alpha, xNew);

    // Compute cost and constraints
    const PerformanceIndex performanceNew = computePerformance(timeDiscretization, initState, xNew, uNew, metricsNew);

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
      metrics = std::move(metricsNew);

      // Prepare step info
      slp::StepInfo stepInfo;
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
  slp::StepInfo stepInfo;
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

slp::Convergence SlpSolver::checkConvergence(int iteration, const PerformanceIndex& baseline, const slp::StepInfo& stepInfo) const {
  using Convergence = slp::Convergence;
  if ((iteration + 1) >= settings_.slpIteration) {
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
