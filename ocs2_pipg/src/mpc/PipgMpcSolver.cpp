#include "ocs2_pipg/mpc/PipgMpcSolver.h"

#include <ocs2_core/control/FeedforwardController.h>

#include <ocs2_sqp/MultipleShootingInitialization.h>
#include <ocs2_sqp/MultipleShootingTranscription.h>

#include <iostream>
#include <numeric>

namespace ocs2 {

PipgMpcSolver::PipgMpcSolver(multiple_shooting::Settings sqpSettings, pipg::Settings pipgSettings,
                             const OptimalControlProblem& optimalControlProblem, const Initializer& initializer)
    : SolverBase(),
      settings_(std::move(sqpSettings)),
      threadPool_(std::max(settings_.nThreads, size_t(1)) - 1, settings_.threadPriority),
      pipgSolver_(pipgSettings),
      pipgSettings_(pipgSettings) {
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

  if (optimalControlProblem.equalityConstraintPtr->empty()) {
    settings_.projectStateInputEqualityConstraints = false;  // True does not make sense if there are no constraints.
  }
}

std::string PipgMpcSolver::getBenchmarkingInformationPIPG() const {
  const auto constructH = constructH_.getTotalInMilliseconds();
  const auto constructG = constructG_.getTotalInMilliseconds();
  const auto GTGMultiplication = GTGMultiplication_.getTotalInMilliseconds();
  const auto preConditioning = preConditioning_.getTotalInMilliseconds();
  const auto lambdaEstimation = lambdaEstimation_.getTotalInMilliseconds();
  const auto sigmaEstimation = sigmaEstimation_.getTotalInMilliseconds();
  const auto pipgRuntime = pipgSolver_.getTotalRunTimeInMilliseconds();

  const auto benchmarkTotal =
      constructH + constructG + GTGMultiplication + preConditioning + lambdaEstimation + sigmaEstimation + pipgRuntime;

  std::stringstream infoStream;
  if (benchmarkTotal > 0.0) {
    const scalar_t inPercent = 100.0;
    infoStream << "\n########################################################################\n";
    infoStream << "The benchmarking is computed over " << constructH_.getNumTimedIntervals() << " iterations. \n";
    infoStream << "PIPG Benchmarking\t       :\tAverage time [ms]   (% of total runtime)\n";
    infoStream << "\tconstructH             :\t" << std::setw(10) << constructH_.getAverageInMilliseconds() << " [ms] \t("
               << constructH / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tconstructG             :\t" << std::setw(10) << constructG_.getAverageInMilliseconds() << " [ms] \t("
               << constructG / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tGTGMultiplication      :\t" << std::setw(10) << GTGMultiplication_.getAverageInMilliseconds() << " [ms] \t("
               << GTGMultiplication / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tpreConditioning        :\t" << std::setw(10) << preConditioning_.getAverageInMilliseconds() << " [ms] \t("
               << preConditioning / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tlambdaEstimation       :\t" << std::setw(10) << lambdaEstimation_.getAverageInMilliseconds() << " [ms] \t("
               << lambdaEstimation / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tsigmaEstimation        :\t" << std::setw(10) << sigmaEstimation_.getAverageInMilliseconds() << " [ms] \t("
               << sigmaEstimation / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tPIPG runTime           :\t" << std::setw(10) << pipgSolver_.getAverageRunTimeInMilliseconds() << " [ms] \t("
               << pipgRuntime / benchmarkTotal * inPercent << "%)\n";
  }
  return infoStream.str();
}

PipgMpcSolver::~PipgMpcSolver() {
  if (settings_.printSolverStatistics) {
    std::cerr << getBenchmarkingInformationPIPG() << "\n" << getBenchmarkingInformation() << std::endl;
  }
}

void PipgMpcSolver::reset() {
  // Clear solution
  primalSolution_ = PrimalSolution();
  performanceIndeces_.clear();

  // reset timers
  numProblems_ = 0;
  totalNumIterations_ = 0;
  linearQuadraticApproximationTimer_.reset();
  solveQpTimer_.reset();
  linesearchTimer_.reset();
  computeControllerTimer_.reset();
}

std::string PipgMpcSolver::getBenchmarkingInformation() const {
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
    infoStream << "\tLQ Approximation   :\t" << std::setw(10) << linearQuadraticApproximationTimer_.getAverageInMilliseconds()
               << " [ms] \t(" << linearQuadraticApproximationTotal / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tSolve QP           :\t" << std::setw(10) << solveQpTimer_.getAverageInMilliseconds() << " [ms] \t("
               << solveQpTotal / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tLinesearch         :\t" << std::setw(10) << linesearchTimer_.getAverageInMilliseconds() << " [ms] \t("
               << linesearchTotal / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tCompute Controller :\t" << std::setw(10) << computeControllerTimer_.getAverageInMilliseconds() << " [ms] \t("
               << computeControllerTotal / benchmarkTotal * inPercent << "%)\n";
  }
  return infoStream.str();
}

const std::vector<PerformanceIndex>& PipgMpcSolver::getIterationsLog() const {
  if (performanceIndeces_.empty()) {
    throw std::runtime_error("[PipgMpcSolver]: No performance log yet, no problem solved yet?");
  } else {
    return performanceIndeces_;
  }
}

void PipgMpcSolver::runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime) {
  if (settings_.printSolverStatus || settings_.printLinesearch) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n+++++++++++++ SQP solver is initialized ++++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
  }

  // Determine time discretization, taking into account event times.
  const auto& eventTimes = this->getReferenceManager().getModeSchedule().eventTimes;
  const auto timeDiscretization = timeDiscretizationWithEvents(initTime, finalTime, settings_.dt, eventTimes);

  // Initialize the state and input
  vector_array_t x, u;
  initializeStateInputTrajectories(initState, timeDiscretization, x, u);

  // Initialize references
  for (auto& ocpDefinition : ocpDefinitions_) {
    const auto& targetTrajectories = this->getReferenceManager().getTargetTrajectories();
    ocpDefinition.targetTrajectoriesPtr = &targetTrajectories;
  }

  // Bookkeeping
  performanceIndeces_.clear();

  int iter = 0;
  multiple_shooting::Convergence convergence = multiple_shooting::Convergence::FALSE;
  while (convergence == multiple_shooting::Convergence::FALSE) {
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
  setPrimalSolution(timeDiscretization, std::move(x), std::move(u));
  computeControllerTimer_.endTimer();

  ++numProblems_;

  if (settings_.printSolverStatus || settings_.printLinesearch) {
    std::cerr << "\nConvergence : " << toString(convergence) << "\n";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n+++++++++++++ SQP solver has terminated ++++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
  }
}

void PipgMpcSolver::runParallel(std::function<void(int)> taskFunction) {
  threadPool_.runParallel(std::move(taskFunction), settings_.nThreads);
}

void PipgMpcSolver::initializeStateInputTrajectories(const vector_t& initState, const std::vector<AnnotatedTime>& timeDiscretization,
                                                     vector_array_t& stateTrajectory, vector_array_t& inputTrajectory) {
  const int N = static_cast<int>(timeDiscretization.size()) - 1;  // // size of the input trajectory
  stateTrajectory.clear();
  stateTrajectory.reserve(N + 1);
  inputTrajectory.clear();
  inputTrajectory.reserve(N);

  // Determine till when to use the previous solution
  scalar_t interpolateStateTill = timeDiscretization.front().time;
  scalar_t interpolateInputTill = timeDiscretization.front().time;
  if (primalSolution_.timeTrajectory_.size() >= 2) {
    interpolateStateTill = primalSolution_.timeTrajectory_.back();
    interpolateInputTill = primalSolution_.timeTrajectory_[primalSolution_.timeTrajectory_.size() - 2];
  }

  // Initial state
  const scalar_t initTime = getIntervalStart(timeDiscretization[0]);
  if (initTime < interpolateStateTill) {
    stateTrajectory.push_back(
        LinearInterpolation::interpolate(initTime, primalSolution_.timeTrajectory_, primalSolution_.stateTrajectory_));
  } else {
    stateTrajectory.push_back(initState);
  }

  for (int i = 0; i < N; i++) {
    if (timeDiscretization[i].event == AnnotatedTime::Event::PreEvent) {
      // Event Node
      inputTrajectory.push_back(vector_t());  // no input at event node
      stateTrajectory.push_back(multiple_shooting::initializeEventNode(timeDiscretization[i].time, stateTrajectory.back()));
    } else {
      // Intermediate node
      const scalar_t time = getIntervalStart(timeDiscretization[i]);
      const scalar_t nextTime = getIntervalEnd(timeDiscretization[i + 1]);
      vector_t input, nextState;
      if (time > interpolateInputTill || nextTime > interpolateStateTill) {  // Using initializer
        std::tie(input, nextState) =
            multiple_shooting::initializeIntermediateNode(*initializerPtr_, time, nextTime, stateTrajectory.back());
      } else {  // interpolate previous solution
        std::tie(input, nextState) = multiple_shooting::initializeIntermediateNode(primalSolution_, time, nextTime, stateTrajectory.back());
      }
      inputTrajectory.push_back(std::move(input));
      stateTrajectory.push_back(std::move(nextState));
    }
  }
}

PipgMpcSolver::OcpSubproblemSolution PipgMpcSolver::getOCPSolution(const vector_t& delta_x0) {
  // Solve the QP
  OcpSubproblemSolution solution;
  auto& deltaXSol = solution.deltaXSol;
  auto& deltaUSol = solution.deltaUSol;

  const bool hasStateInputConstraints = !ocpDefinitions_.front().equalityConstraintPtr->empty();
  if (hasStateInputConstraints && !settings_.projectStateInputEqualityConstraints) {
    throw std::runtime_error("[PipgMpcSolver] Please project State-Input equality constraints.");
  }

  // without constraints, or when using projection, we have an unconstrained QP.
  pipgSolver_.resize(pipg::extractSizesFromProblem(dynamics_, cost_, nullptr));

  constructH_.startTimer();
  constructH_.endTimer();

  constructG_.startTimer();
  constructG_.endTimer();

  vector_array_t D, E;
  vector_array_t scalingVectors;
  scalar_t c;

  preConditioning_.startTimer();
  pipgSolver_.preConditioningInPlaceInParallel(delta_x0, dynamics_, cost_, pipgSettings_.numScaling, D, E, scalingVectors, c,
                                               Eigen::SparseMatrix<scalar_t>(), vector_t(), Eigen::SparseMatrix<scalar_t>());
  preConditioning_.endTimer();

  lambdaEstimation_.startTimer();
  vector_t rowwiseAbsSumH = pipgSolver_.HAbsRowSumInParallel(cost_);
  scalar_t lambdaScaled = rowwiseAbsSumH.maxCoeff();
  lambdaEstimation_.endTimer();

  GTGMultiplication_.startTimer();
  vector_t rowwiseAbsSumGGT;
  pipgSolver_.GGTAbsRowSumInParallel(dynamics_, nullptr, &scalingVectors, rowwiseAbsSumGGT);
  GTGMultiplication_.endTimer();

  sigmaEstimation_.startTimer();
  scalar_t sigmaScaled = rowwiseAbsSumGGT.maxCoeff();
  sigmaEstimation_.endTimer();

  scalar_t maxScalingFactor = -1;
  for (auto& v : D) {
    if (v.size() != 0) {
      maxScalingFactor = std::max(maxScalingFactor, v.maxCoeff());
    }
  }
  scalar_t muEstimated = pipgSettings_.lowerBoundH * c * maxScalingFactor * maxScalingFactor;

  vector_array_t EInv(E.size());
  std::transform(E.begin(), E.end(), EInv.begin(), [](const vector_t& v) { return v.cwiseInverse(); });
  Pipg::SolverStatus pipgStatus =
      pipgSolver_.solveOCPInParallel(delta_x0, dynamics_, cost_, nullptr, scalingVectors, &EInv, muEstimated, lambdaScaled, sigmaScaled,
                                     ScalarFunctionQuadraticApproximation(), VectorFunctionLinearApproximation());
  pipgSolver_.getStateInputTrajectoriesSolution(deltaXSol, deltaUSol);

  // To determine if the solution is a descent direction for the cost: compute gradient(cost)' * [dx; du]
  solution.armijoDescentMetric = 0.0;
  for (int i = 0; i < cost_.size(); i++) {
    if (cost_[i].dfdx.size() > 0) {
      solution.armijoDescentMetric += cost_[i].dfdx.dot(deltaXSol[i]);
    }
    if (cost_[i].dfdu.size() > 0) {
      solution.armijoDescentMetric += cost_[i].dfdu.dot(deltaUSol[i]);
    }
  }

  pipgSolver_.descaleSolution(D, deltaXSol, deltaUSol);

  // remap the tilde delta u to real delta u
  if (settings_.projectStateInputEqualityConstraints) {
    vector_t tmp;  // 1 temporary for re-use.
    for (int i = 0; i < deltaUSol.size(); i++) {
      if (constraintsProjection_[i].f.size() > 0) {
        tmp.noalias() = constraintsProjection_[i].dfdu * deltaUSol[i];
        deltaUSol[i] = tmp + constraintsProjection_[i].f;
        deltaUSol[i].noalias() += constraintsProjection_[i].dfdx * deltaXSol[i];
      }
    }
  }

  return solution;
}

void PipgMpcSolver::setPrimalSolution(const std::vector<AnnotatedTime>& time, vector_array_t&& x, vector_array_t&& u) {
  // Clear old solution
  primalSolution_.clear();

  // Correct for missing inputs at PreEvents
  for (int i = 0; i < time.size(); ++i) {
    if (time[i].event == AnnotatedTime::Event::PreEvent && i > 0) {
      u[i] = u[i - 1];
    }
  }

  // Construct nominal time, state and input trajectories
  primalSolution_.stateTrajectory_ = std::move(x);
  u.push_back(u.back());  // Repeat last input to make equal length vectors
  primalSolution_.inputTrajectory_ = std::move(u);
  primalSolution_.timeTrajectory_.reserve(time.size());
  for (size_t i = 0; i < time.size(); i++) {
    primalSolution_.timeTrajectory_.push_back(time[i].time);
    if (time[i].event == AnnotatedTime::Event::PreEvent) {
      primalSolution_.postEventIndices_.push_back(i + 1);
    }
  }
  primalSolution_.modeSchedule_ = this->getReferenceManager().getModeSchedule();

  primalSolution_.controllerPtr_.reset(new FeedforwardController(primalSolution_.timeTrajectory_, primalSolution_.inputTrajectory_));
}

PerformanceIndex PipgMpcSolver::setupQuadraticSubproblem(const std::vector<AnnotatedTime>& time, const vector_t& initState,
                                                         const vector_array_t& x, const vector_array_t& u) {
  // Problem horizon
  const int N = static_cast<int>(time.size()) - 1;

  std::vector<PerformanceIndex> performance(settings_.nThreads, PerformanceIndex());
  dynamics_.resize(N);
  cost_.resize(N + 1);
  constraints_.resize(N + 1);
  constraintsProjection_.resize(N);

  std::atomic_int timeIndex{0};
  auto parallelTask = [&](int workerId) {
    // Get worker specific resources
    OptimalControlProblem& ocpDefinition = ocpDefinitions_[workerId];
    PerformanceIndex workerPerformance;  // Accumulate performance in local variable
    const bool projection = settings_.projectStateInputEqualityConstraints;

    int i = timeIndex++;
    while (i < N) {
      if (time[i].event == AnnotatedTime::Event::PreEvent) {
        // Event node
        auto result = multiple_shooting::setupEventNode(ocpDefinition, time[i].time, x[i], x[i + 1]);
        workerPerformance += result.performance;
        dynamics_[i] = std::move(result.dynamics);
        cost_[i] = std::move(result.cost);
        constraints_[i] = std::move(result.constraints);
        constraintsProjection_[i] = VectorFunctionLinearApproximation::Zero(0, x[i].size(), 0);
      } else {
        // Normal, intermediate node
        const scalar_t ti = getIntervalStart(time[i]);
        const scalar_t dt = getIntervalDuration(time[i], time[i + 1]);
        auto result =
            multiple_shooting::setupIntermediateNode(ocpDefinition, sensitivityDiscretizer_, projection, ti, dt, x[i], x[i + 1], u[i]);
        workerPerformance += result.performance;
        dynamics_[i] = std::move(result.dynamics);
        cost_[i] = std::move(result.cost);
        constraints_[i] = std::move(result.constraints);
        constraintsProjection_[i] = std::move(result.constraintsProjection);
      }

      i = timeIndex++;
    }

    if (i == N) {  // Only one worker will execute this
      const scalar_t tN = getIntervalStart(time[N]);
      auto result = multiple_shooting::setupTerminalNode(ocpDefinition, tN, x[N]);
      workerPerformance += result.performance;
      cost_[i] = std::move(result.cost);
      constraints_[i] = std::move(result.constraints);
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

PerformanceIndex PipgMpcSolver::computePerformance(const std::vector<AnnotatedTime>& time, const vector_t& initState,
                                                   const vector_array_t& x, const vector_array_t& u) {
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

scalar_t PipgMpcSolver::trajectoryNorm(const vector_array_t& v) {
  scalar_t norm = 0.0;
  for (const auto& vi : v) {
    norm += vi.squaredNorm();
  }
  return std::sqrt(norm);
}

scalar_t PipgMpcSolver::totalConstraintViolation(const PerformanceIndex& performance) const {
  return std::sqrt(performance.dynamicsViolationSSE + performance.equalityConstraintsSSE);
}

multiple_shooting::StepInfo PipgMpcSolver::takeStep(const PerformanceIndex& baseline, const std::vector<AnnotatedTime>& timeDiscretization,
                                                    const vector_t& initState, const OcpSubproblemSolution& subproblemSolution,
                                                    vector_array_t& x, vector_array_t& u) {
  using StepType = multiple_shooting::StepInfo::StepType;

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
  const scalar_t baselineConstraintViolation = totalConstraintViolation(baseline);

  // Update norm
  const auto& dx = subproblemSolution.deltaXSol;
  const auto& du = subproblemSolution.deltaUSol;
  const scalar_t deltaUnorm = trajectoryNorm(du);
  const scalar_t deltaXnorm = trajectoryNorm(dx);

  // Prepare step info
  multiple_shooting::StepInfo stepInfo;

  scalar_t alpha = 1.0;
  vector_array_t xNew(x.size());
  vector_array_t uNew(u.size());
  do {
    // Compute step
    for (int i = 0; i < u.size(); i++) {
      if (du[i].size() > 0) {  // account for absence of inputs at events.
        uNew[i] = u[i] + alpha * du[i];
      }
    }
    for (int i = 0; i < x.size(); i++) {
      xNew[i] = x[i] + alpha * dx[i];
    }

    // Compute cost and constraints
    const PerformanceIndex performanceNew = computePerformance(timeDiscretization, initState, xNew, uNew);
    const scalar_t newConstraintViolation = totalConstraintViolation(performanceNew);

    // Step acceptance and record step type
    const bool stepAccepted = [&]() {
      if (newConstraintViolation > settings_.g_max) {
        // High constraint violation. Only accept decrease in constraints.
        stepInfo.stepType = StepType::CONSTRAINT;
        return newConstraintViolation < ((1.0 - settings_.gamma_c) * baselineConstraintViolation);
      } else if (newConstraintViolation < settings_.g_min && baselineConstraintViolation < settings_.g_min &&
                 subproblemSolution.armijoDescentMetric < 0.0) {
        // With low violation and having a descent direction, require the armijo condition.
        stepInfo.stepType = StepType::COST;
        return performanceNew.merit < (baseline.merit + settings_.armijoFactor * alpha * subproblemSolution.armijoDescentMetric);
      } else {
        // Medium violation: either merit or constraints decrease (with small gamma_c mixing of old constraints)
        stepInfo.stepType = StepType::DUAL;
        return performanceNew.merit < (baseline.merit - settings_.gamma_c * baselineConstraintViolation) ||
               newConstraintViolation < ((1.0 - settings_.gamma_c) * baselineConstraintViolation);
      }
    }();

    if (settings_.printLinesearch) {
      std::cerr << "Step size: " << alpha << ", Step Type: " << toString(stepInfo.stepType)
                << (stepAccepted ? std::string{" (Accepted)"} : std::string{" (Rejected)"}) << "\n";
      std::cerr << "|dx| = " << alpha * deltaXnorm << "\t|du| = " << alpha * deltaUnorm << "\n";
      std::cerr << performanceNew << "\n";
    }

    if (stepAccepted) {  // Return if step accepted
      x = std::move(xNew);
      u = std::move(uNew);

      stepInfo.stepSize = alpha;
      stepInfo.dx_norm = alpha * deltaXnorm;
      stepInfo.du_norm = alpha * deltaUnorm;
      stepInfo.performanceAfterStep = performanceNew;
      stepInfo.totalConstraintViolationAfterStep = newConstraintViolation;
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
  stepInfo.stepSize = 0.0;
  stepInfo.stepType = StepType::ZERO;
  stepInfo.dx_norm = 0.0;
  stepInfo.du_norm = 0.0;
  stepInfo.performanceAfterStep = baseline;
  stepInfo.totalConstraintViolationAfterStep = baselineConstraintViolation;

  if (settings_.printLinesearch) {
    std::cerr << "[Linesearch terminated] Step size: " << stepInfo.stepSize << ", Step Type: " << toString(stepInfo.stepType) << "\n";
  }

  return stepInfo;
}

multiple_shooting::Convergence PipgMpcSolver::checkConvergence(int iteration, const PerformanceIndex& baseline,
                                                               const multiple_shooting::StepInfo& stepInfo) const {
  using Convergence = multiple_shooting::Convergence;
  if ((iteration + 1) >= settings_.sqpIteration) {
    // Converged because the next iteration would exceed the specified number of iterations
    return Convergence::ITERATIONS;
  } else if (stepInfo.stepSize < settings_.alpha_min) {
    // Converged because step size is below the specified minimum
    return Convergence::STEPSIZE;
  } else if (std::abs(stepInfo.performanceAfterStep.merit - baseline.merit) < settings_.costTol &&
             totalConstraintViolation(stepInfo.performanceAfterStep) < settings_.g_min) {
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
