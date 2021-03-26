//
// Created by rgrandia on 09.11.20.
//

#include "ocs2_sqp/MultipleShootingSolver.h"

#include <iostream>
#include <numeric>

#include <ocs2_core/OCS2NumericTraits.h>
#include <ocs2_core/constraint/RelaxedBarrierPenalty.h>
#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_oc/approximate_model/ChangeOfInputVariables.h>
#include <ocs2_sqp/ConstraintProjection.h>

namespace ocs2 {

MultipleShootingSolver::MultipleShootingSolver(MultipleShootingSolverSettings settings, const SystemDynamicsBase* systemDynamicsPtr,
                                               const CostFunctionBase* costFunctionPtr, const ConstraintBase* constraintPtr,
                                               const CostFunctionBase* terminalCostFunctionPtr,
                                               const SystemOperatingTrajectoriesBase* operatingTrajectoriesPtr)
    : SolverBase(), terminalCostFunctionPtr_(nullptr), settings_(std::move(settings)), totalNumIterations_(0), performanceIndeces_() {
  // Multithreading, set up threadpool for N-1 helpers, our main thread is the N-th one.
  if (settings_.nThreads > 1) {
    threadPoolPtr_.reset(new ThreadPool(settings_.nThreads - 1, settings_.threadPriority));
  }
  Eigen::setNbThreads(1);  // No multithreading within Eigen.
  Eigen::initParallel();

  discretizer_ = selectDynamicsDiscretization(settings.integratorType);
  sensitivityDiscretizer_ = selectDynamicsSensitivityDiscretization(settings.integratorType);

  for (int w = 0; w < settings.nThreads; w++) {
    systemDynamicsPtr_.emplace_back(systemDynamicsPtr->clone());
    costFunctionPtr_.emplace_back(costFunctionPtr->clone());
    if (constraintPtr != nullptr) {
      constraintPtr_.emplace_back(constraintPtr->clone());
    } else {
      constraintPtr_.emplace_back(nullptr);
    }
  }

  if (constraintPtr != nullptr && settings_.inequalityConstraintMu > 0) {
    penaltyPtr_.reset(new RelaxedBarrierPenalty(settings_.inequalityConstraintMu, settings_.inequalityConstraintDelta));
  }

  if (terminalCostFunctionPtr != nullptr) {
    terminalCostFunctionPtr_.reset(terminalCostFunctionPtr->clone());
  }

  if (operatingTrajectoriesPtr != nullptr) {
    operatingTrajectoriesPtr_.reset(operatingTrajectoriesPtr->clone());
  }
}

MultipleShootingSolver::~MultipleShootingSolver() {
  if (settings_.printSolverStatistics) {
    std::cerr << getBenchmarkingInformation() << std::endl;
  }
}

void MultipleShootingSolver::reset() {
  // Clear solution
  primalSolution_ = PrimalSolution();
  performanceIndeces_.clear();

  // reset timers
  totalNumIterations_ = 0;
  linearQuadraticApproximationTimer_.reset();
  solveQpTimer_.reset();
  computeControllerTimer_.reset();
}

std::string MultipleShootingSolver::getBenchmarkingInformation() const {
  const auto linearQuadraticApproximationTotal = linearQuadraticApproximationTimer_.getTotalInMilliseconds();
  const auto solveQpTotal = solveQpTimer_.getTotalInMilliseconds();
  const auto computeControllerTotal = computeControllerTimer_.getTotalInMilliseconds();

  const auto benchmarkTotal = linearQuadraticApproximationTotal + solveQpTotal + computeControllerTotal;

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
    infoStream << "\tCompute Controller :\t" << computeControllerTimer_.getAverageInMilliseconds() << " [ms] \t\t("
               << computeControllerTotal / benchmarkTotal * inPercent << "%)\n";
  }
  return infoStream.str();
}

const std::vector<PerformanceIndex>& MultipleShootingSolver::getIterationsLog() const {
  if (performanceIndeces_.empty()) {
    throw std::runtime_error("[MultipleShootingSolver]: No performance log yet, no problem solved yet?");
  } else {
    return performanceIndeces_;
  }
}

void MultipleShootingSolver::runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime,
                                     const scalar_array_t& partitioningTimes) {
  if (settings_.printSolverStatus || settings_.printLinesearch) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n+++++++++++++ SQP solver is initialized ++++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
  }

  // Determine time discretization, taking into account event times.
  scalar_array_t timeDiscretization = timeDiscretizationWithEvents(initTime, finalTime, settings_.dt, this->getModeSchedule().eventTimes,
                                                                   OCS2NumericTraits<scalar_t>::limitEpsilon());
  const int N = static_cast<int>(timeDiscretization.size()) - 1;

  // Initialize the state and input
  vector_array_t x = initializeStateTrajectory(initState, timeDiscretization, N);
  vector_array_t u = initializeInputTrajectory(timeDiscretization, x, N);

  // Initialize cost
  for (auto& cost : costFunctionPtr_) {
    cost->setCostDesiredTrajectoriesPtr(&this->getCostDesiredTrajectories());
  }
  if (terminalCostFunctionPtr_) {
    terminalCostFunctionPtr_->setCostDesiredTrajectoriesPtr(&this->getCostDesiredTrajectories());
  }

  // Bookkeeping
  performanceIndeces_.clear();

  for (int iter = 0; iter < settings_.sqpIteration; iter++) {
    if (settings_.printSolverStatus || settings_.printLinesearch) {
      std::cerr << "\nSQP iteration: " << iter << "\n";
    }
    // Make QP approximation
    linearQuadraticApproximationTimer_.startTimer();
    performanceIndeces_.push_back(setupQuadraticSubproblem(timeDiscretization, initState, x, u));
    linearQuadraticApproximationTimer_.endTimer();

    // Solve QP
    solveQpTimer_.startTimer();
    const vector_t delta_x0 = initState - x[0];
    vector_array_t delta_x;
    vector_array_t delta_u;
    std::tie(delta_x, delta_u) = getOCPSolution(delta_x0);
    solveQpTimer_.endTimer();

    // Apply step
    computeControllerTimer_.startTimer();
    bool converged = takeStep(performanceIndeces_.back(), timeDiscretization, initState, delta_x, delta_u, x, u);
    computeControllerTimer_.endTimer();

    totalNumIterations_++;
    if (converged) {
      break;
    }
  }

  // Store result in PrimalSolution. time, state , input
  primalSolution_.timeTrajectory_ = std::move(timeDiscretization);
  primalSolution_.stateTrajectory_ = std::move(x);
  primalSolution_.inputTrajectory_ = std::move(u);
  primalSolution_.inputTrajectory_.push_back(primalSolution_.inputTrajectory_.back());  // repeat last input to make equal length vectors
  primalSolution_.modeSchedule_ = this->getModeSchedule();

  if (constraintPtr_.front() && settings_.qr_decomp) {
    // TODO: Add feedback in u_tilde space. Currently only have feedback arising from stateInputEquality
    vector_array_t uff;
    matrix_array_t controllerGain;
    uff.reserve(N + 1);
    controllerGain.reserve(N + 1);
    for (int i = 0; i < N; i++) {
      // Linear controller has convention u = uff + K * x;
      // We computed u = u'(t) + K (x - x'(t));
      // >> uff = u'(t) - K x'(t)
      uff.push_back(primalSolution_.inputTrajectory_[i]);
      uff.back().noalias() -= constraints_[i].dfdx * primalSolution_.stateTrajectory_[i];
      controllerGain.push_back(std::move(constraints_[i].dfdx));  // Steal! the feedback matrix, don't use after this.
    }
    // Copy last one the get correct length
    uff.push_back(uff.back());
    controllerGain.push_back(controllerGain.back());
    primalSolution_.controllerPtr_.reset(new LinearController(primalSolution_.timeTrajectory_, std::move(uff), std::move(controllerGain)));
  } else {
    primalSolution_.controllerPtr_.reset(new FeedforwardController(primalSolution_.timeTrajectory_, primalSolution_.inputTrajectory_));
  }

  if (settings_.printSolverStatus || settings_.printLinesearch) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n+++++++++++++ SQP solver has terminated ++++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
  }
}

void MultipleShootingSolver::runParallel(std::function<void(int)> taskFunction) {
  // Launch tasks in helper threads
  std::vector<std::future<void>> futures{};
  if (threadPoolPtr_) {
    int numHelpers = settings_.nThreads - 1;
    futures.reserve(numHelpers);
    for (int i = 0; i < numHelpers; i++) {
      futures.emplace_back(threadPoolPtr_->run(taskFunction));
    }
  }
  // Execute one instance in this thread.
  taskFunction(0);

  // Wait for helpers to finish.
  for (auto&& fut : futures) {
    fut.get();
  }
}

vector_array_t MultipleShootingSolver::initializeInputTrajectory(const scalar_array_t& timeDiscretization,
                                                                 const vector_array_t& stateTrajectory, int N) const {
  const scalar_t interpolateTill = (totalNumIterations_ > 0) ? primalSolution_.timeTrajectory_.back() : timeDiscretization.front();

  vector_array_t u;
  u.reserve(N);
  for (int i = 0; i < N; i++) {
    const scalar_t ti = timeDiscretization[i];
    if (ti < interpolateTill) {
      // Interpolate previous input trajectory
      u.emplace_back(
          LinearInterpolation::interpolate(timeDiscretization[i], primalSolution_.timeTrajectory_, primalSolution_.inputTrajectory_));
    } else {
      // No previous control at this time-point -> fall back to heuristics
      if (operatingTrajectoriesPtr_) {
        // Ask for operating trajectory between t[k] and t[k+1]. Take the returned input at t[k] as our heuristic.
        const scalar_t tNext = timeDiscretization[i + 1];
        scalar_array_t timeArray;
        vector_array_t stateArray;
        vector_array_t inputArray;
        operatingTrajectoriesPtr_->getSystemOperatingTrajectories(stateTrajectory[i], ti, tNext, timeArray, stateArray, inputArray, false);
        u.push_back(std::move(inputArray.front()));
      } else {  // No information at all. Set inputs to zero.
        u.emplace_back(vector_t::Zero(settings_.n_input));
      }
    }
  }

  return u;
}

vector_array_t MultipleShootingSolver::initializeStateTrajectory(const vector_t& initState, const scalar_array_t& timeDiscretization,
                                                                 int N) const {
  if (totalNumIterations_ == 0) {  // first iteration
    return vector_array_t(N + 1, initState);
  } else {  // interpolation of previous solution
    vector_array_t x;
    x.reserve(N + 1);
    x.push_back(initState);  // Force linearization of the first node around the current state
    for (int i = 1; i < (N + 1); i++) {
      x.emplace_back(
          LinearInterpolation::interpolate(timeDiscretization[i], primalSolution_.timeTrajectory_, primalSolution_.stateTrajectory_));
    }
    return x;
  }
}

std::pair<vector_array_t, vector_array_t> MultipleShootingSolver::getOCPSolution(const vector_t& delta_x0) {
  // Solve the QP
  vector_array_t deltaXSol;
  vector_array_t deltaUSol;
  hpipm_status status;
  if (constraintPtr_.front() && !settings_.qr_decomp) {
    status = hpipmInterface_.solve(delta_x0, dynamics_, cost_, &constraints_, deltaXSol, deltaUSol, settings_.printSolverStatus);
  } else {  // without constraints, or when using QR decomposition, we have an unconstrained QP.
    status = hpipmInterface_.solve(delta_x0, dynamics_, cost_, nullptr, deltaXSol, deltaUSol, settings_.printSolverStatus);
  }

  if (status != hpipm_status::SUCCESS) {
    throw std::runtime_error("[MultipleShootingSolver] Failed to solve QP");
  }

  // remap the tilde delta u to real delta u
  if (constraintPtr_.front() && settings_.qr_decomp) {
    for (int i = 0; i < deltaUSol.size(); i++) {
      deltaUSol[i] = constraints_[i].dfdu * deltaUSol[i];  // creates a temporary because of alias
      deltaUSol[i].noalias() += constraints_[i].dfdx * deltaXSol[i];
      deltaUSol[i] += constraints_[i].f;
    }
  }

  return {deltaXSol, deltaUSol};
}

PerformanceIndex MultipleShootingSolver::setupQuadraticSubproblem(const scalar_array_t& time, const vector_t& initState,
                                                                  const vector_array_t& x, const vector_array_t& u) {
  // Problem horizon
  const int N = static_cast<int>(time.size()) - 1;

  // Set up for constant state input size. Will be adapted based on constraint handling.
  HpipmInterface::OcpSize ocpSize(N, settings_.n_state, settings_.n_input);

  std::vector<PerformanceIndex> performance(settings_.nThreads, PerformanceIndex());
  dynamics_.resize(N);
  cost_.resize(N + 1);
  constraints_.resize(N + 1);

  std::atomic_int workerId{0};
  std::atomic_int timeIndex{0};
  auto parallelTask = [&](int) {
    // Get worker specific resources
    int thisWorker = workerId++;  // assign worker ID (atomic)
    SystemDynamicsBase& systemDynamics = *systemDynamicsPtr_[thisWorker];
    CostFunctionBase& costFunction = *costFunctionPtr_[thisWorker];
    ConstraintBase* constraintPtr = constraintPtr_[thisWorker].get();
    PerformanceIndex workerPerformance;  // Accumulate performance in local variable
    const bool qpDecomp = settings_.qr_decomp;

    int i = timeIndex++;
    while (i < N) {
      VectorFunctionLinearApproximation dynamics;
      ScalarFunctionQuadraticApproximation cost;
      VectorFunctionLinearApproximation constraints;

      const scalar_t ti = time[i];
      const scalar_t dt = time[i + 1] - ti;

      setupIntermediateNode(systemDynamics, sensitivityDiscretizer_, costFunction, constraintPtr, penaltyPtr_.get(), qpDecomp, ti, dt, x[i],
                            x[i + 1], u[i], workerPerformance, dynamics, cost, constraints);

      dynamics_[i] = std::move(dynamics);
      cost_[i] = std::move(cost);
      constraints_[i] = std::move(constraints);
      i = timeIndex++;
    }

    if (i == N) {  // Only one worker will execute this
      CostFunctionBase* terminalCostFunctionPtr = terminalCostFunctionPtr_.get();
      setTerminalNode(terminalCostFunctionPtr, time[N], x[N], performance.front(), cost_[N], constraints_[N]);
    }

    performance[thisWorker] = workerPerformance;
  };
  runParallel(parallelTask);

  // Account for init state in performance
  performance.front().stateEqConstraintISE += (initState - x.front()).squaredNorm();

  // determine sizes
  for (int i = 0; i < N; i++) {
    if (constraintPtr_.front() != nullptr) {
      if (settings_.qr_decomp) {
        ocpSize.numInputs[i] = constraints_[i].dfdu.cols();  // obtain size of u_tilde from constraint projection.
      } else {
        ocpSize.numIneqConstraints[i] = constraints_[i].f.rows();  // Declare as general inequalities
      }
    }
  }

  // Prepare solver size
  hpipmInterface_.resize(std::move(ocpSize));

  // Sum performance of the threads
  return std::accumulate(std::next(performance.begin()), performance.end(), performance.front());
}

void MultipleShootingSolver::setupIntermediateNode(SystemDynamicsBase& systemDynamics,
                                                   DynamicsSensitivityDiscretizer& sensitivityDiscretizer, CostFunctionBase& costFunction,
                                                   ConstraintBase* constraintPtr, PenaltyBase* penaltyPtr,
                                                   bool projectStateInputEqualityConstraints, scalar_t t, scalar_t dt, const vector_t& x,
                                                   const vector_t& x_next, const vector_t& u, PerformanceIndex& performance,
                                                   VectorFunctionLinearApproximation& dynamics, ScalarFunctionQuadraticApproximation& cost,
                                                   VectorFunctionLinearApproximation& constraints) {
  // Dynamics
  // Discretization returns // x_{k+1} = A_{k} * dx_{k} + B_{k} * du_{k} + b_{k}
  dynamics = sensitivityDiscretizer(systemDynamics, t, x, u, dt);
  dynamics.f -= x_next;  // make it dx_{k+1} = ...
  performance.stateEqConstraintISE += dt * dynamics.f.squaredNorm();

  // Costs: Approximate the integral with forward euler (correct for dt after adding penalty)
  cost = costFunction.costQuadraticApproximation(t, x, u);
  performance.totalCost += dt * cost.f;

  if (constraintPtr != nullptr) {
    // Inequalities as penalty
    if (penaltyPtr != nullptr) {
      const auto ineqConstraints = constraintPtr->inequalityConstraintQuadraticApproximation(t, x, u);
      if (ineqConstraints.f.rows() > 0) {
        auto penaltyCost = penaltyPtr->penaltyCostQuadraticApproximation(ineqConstraints);
        cost += penaltyCost;  // add to cost before potential projection.
        performance.inequalityConstraintISE += dt * ineqConstraints.f.cwiseMin(0.0).squaredNorm();
        performance.inequalityConstraintPenalty += dt * penaltyCost.f;
      }
    }

    // C_{k} * dx_{k} + D_{k} * du_{k} + e_{k} = 0
    constraints = constraintPtr->stateInputEqualityConstraintLinearApproximation(t, x, u);
    performance.stateInputEqConstraintISE += dt * constraints.f.squaredNorm();
    if (projectStateInputEqualityConstraints) {  // Handle equality constraints using projection.
      // Projection stored instead of constraint, // TODO: benchmark between lu and qr method. LU seems slightly faster.
      constraints = luConstraintProjection(constraints);

      // Adapt dynamics and cost
      changeOfInputVariables(dynamics, constraints.dfdu, constraints.dfdx, constraints.f);
      changeOfInputVariables(cost, constraints.dfdu, constraints.dfdx, constraints.f);
    }
  }

  // Costs: Approximate the integral with forward euler  (correct for dt HERE, after adding penalty)
  cost.dfdxx *= dt;
  cost.dfdux *= dt;
  cost.dfduu *= dt;
  cost.dfdx *= dt;
  cost.dfdu *= dt;
  cost.f *= dt;

  // Merit
  performance.merit = performance.totalCost + performance.inequalityConstraintPenalty;
}

void MultipleShootingSolver::setTerminalNode(CostFunctionBase* terminalCostFunctionPtr, scalar_t t, const vector_t& x,
                                             PerformanceIndex& performance, ScalarFunctionQuadraticApproximation& cost,
                                             VectorFunctionLinearApproximation& constraints) {
  // Terminal conditions
  if (terminalCostFunctionPtr != nullptr) {
    cost = terminalCostFunctionPtr->finalCostQuadraticApproximation(t, x);
    performance.totalCost += cost.f;
  } else {
    cost = ScalarFunctionQuadraticApproximation::Zero(x.size(), 0);
  }

  constraints = VectorFunctionLinearApproximation::Zero(0, x.size(), 0);
}

PerformanceIndex MultipleShootingSolver::computePerformance(const scalar_array_t& time, const vector_t& initState, const vector_array_t& x,
                                                            const vector_array_t& u) {
  // Problem horizon
  const int N = static_cast<int>(time.size()) - 1;

  std::vector<PerformanceIndex> performance(settings_.nThreads, PerformanceIndex());
  std::atomic_int workerId{0};
  std::atomic_int timeIndex{0};
  auto parallelTask = [&](int) {
    // Get worker specific resources
    int thisWorker = workerId++;  // assign worker ID (atomic)
    SystemDynamicsBase& systemDynamics = *systemDynamicsPtr_[thisWorker];
    CostFunctionBase& costFunction = *costFunctionPtr_[thisWorker];
    ConstraintBase* constraintPtr = constraintPtr_[thisWorker].get();
    PerformanceIndex workerPerformance;  // Accumulate performance in local variable

    int i = timeIndex++;
    while (i < N) {
      const scalar_t ti = time[i];
      const scalar_t dt = time[i + 1] - ti;

      computePerformance(systemDynamics, discretizer_, costFunction, constraintPtr, penaltyPtr_.get(), ti, dt, x[i], x[i + 1], u[i],
                         workerPerformance);

      i = timeIndex++;
    }

    if (i == N && terminalCostFunctionPtr_) {  // Only one worker will execute this
      workerPerformance.totalCost += terminalCostFunctionPtr_->finalCost(time[N], x[N]);
    }

    performance[thisWorker] = workerPerformance;
  };
  runParallel(parallelTask);

  // Account for init state in performance
  performance.front().stateEqConstraintISE += (initState - x.front()).squaredNorm();

  // Sum performance of the threads
  return std::accumulate(std::next(performance.begin()), performance.end(), performance.front());
}

void MultipleShootingSolver::computePerformance(SystemDynamicsBase& systemDynamics, DynamicsDiscretizer& discretizer,
                                                CostFunctionBase& costFunction, ConstraintBase* constraintPtr, PenaltyBase* penaltyPtr,
                                                scalar_t t, scalar_t dt, const vector_t& x, const vector_t& x_next, const vector_t& u,
                                                PerformanceIndex& performance) {
  // Dynamics
  const vector_t dynamicsGap = discretizer(systemDynamics, t, x, u, dt) - x_next;
  performance.stateEqConstraintISE += dt * dynamicsGap.squaredNorm();

  // Costs
  performance.totalCost += dt * costFunction.cost(t, x, u);

  if (constraintPtr != nullptr) {
    const vector_t constraints = constraintPtr->stateInputEqualityConstraint(t, x, u);
    performance.stateInputEqConstraintISE += dt * constraints.squaredNorm();

    // Inequalities as penalty
    if (penaltyPtr) {
      const vector_t ineqConstraints = constraintPtr->inequalityConstraint(t, x, u);
      if (ineqConstraints.rows() > 0) {
        scalar_t penaltyCost = penaltyPtr->penaltyCost(ineqConstraints);
        performance.inequalityConstraintISE += dt * ineqConstraints.cwiseMin(0.0).squaredNorm();
        performance.inequalityConstraintPenalty += dt * penaltyCost;
      }
    }
  }

  // Merit
  performance.merit = performance.totalCost + performance.inequalityConstraintPenalty;
}

bool MultipleShootingSolver::takeStep(const PerformanceIndex& baseline, const scalar_array_t& timeDiscretization, const vector_t& initState,
                                      const vector_array_t& dx, const vector_array_t& du, vector_array_t& x, vector_array_t& u) {
  /*
   * Filter linesearch based on:
   * "On the implementation of an interior-point filter line-search algorithm for large-scale nonlinear programming"
   * https://link.springer.com/article/10.1007/s10107-004-0559-y
   */
  if (settings_.printLinesearch) {
    std::cerr << std::setprecision(9) << std::fixed;
    std::cerr << "\n=== Linesearch ===\n";
    std::cerr << "Baseline:\n";
    std::cerr << "\tMerit: " << baseline.merit << "\t DynamicsISE: " << baseline.stateEqConstraintISE
              << "\t StateInputISE: " << baseline.stateInputEqConstraintISE << "\t IneqISE: " << baseline.inequalityConstraintISE
              << "\t Penalty: " << baseline.inequalityConstraintPenalty << "\n";
  }

  // Some settings // TODO: Make parameters
  const scalar_t alpha_decay = 0.5;
  const scalar_t alpha_min = 1e-4;
  const scalar_t gamma_c = 1e-6;
  const scalar_t g_max = 1e6;
  const scalar_t g_min = 1e-6;
  const scalar_t costTol = 1e-4;

  const int N = static_cast<int>(timeDiscretization.size()) - 1;
  scalar_t baselineConstraintViolation =
      std::sqrt(baseline.stateEqConstraintISE + baseline.stateInputEqConstraintISE + baseline.inequalityConstraintISE);

  // Update norm
  scalar_t deltaUnorm = 0.0;
  for (int i = 0; i < N; i++) {
    deltaUnorm += du[i].squaredNorm();
  }
  deltaUnorm = std::sqrt(deltaUnorm);
  scalar_t deltaXnorm = 0.0;
  for (int i = 0; i < (N + 1); i++) {
    deltaXnorm += dx[i].squaredNorm();
  }
  deltaXnorm = std::sqrt(deltaXnorm);

  scalar_t alpha = 1.0;
  vector_array_t xNew(x.size());
  vector_array_t uNew(u.size());
  while (alpha > alpha_min) {
    // Compute step
    for (int i = 0; i < N; i++) {
      uNew[i] = u[i] + alpha * du[i];
    }
    for (int i = 0; i < (N + 1); i++) {
      xNew[i] = x[i] + alpha * dx[i];
    }

    // Compute cost and constraints
    PerformanceIndex performanceNew = computePerformance(timeDiscretization, initState, xNew, uNew);
    scalar_t newConstraintViolation =
        std::sqrt(performanceNew.stateEqConstraintISE + performanceNew.stateInputEqConstraintISE + performanceNew.inequalityConstraintISE);

    bool stepAccepted = [&]() {
      if (newConstraintViolation > g_max) {
        return false;
      } else if (newConstraintViolation < g_min) {
        // With low violation only care about cost, reference paper implements here armijo condition
        return (performanceNew.merit < baseline.merit);
      } else {
        // Medium violation: either merit or constraints decrease (with small gamma_c mixing of old constraints)
        return performanceNew.merit < (baseline.merit - gamma_c * baselineConstraintViolation) ||
               newConstraintViolation < ((1.0 - gamma_c) * baselineConstraintViolation);
      }
    }();

    if (settings_.printLinesearch) {
      std::cerr << "Stepsize = " << alpha << (stepAccepted ? std::string{" (Accepted)"} : std::string{" (Rejected)"}) << "\n";
      std::cerr << "|dx| = " << alpha * deltaXnorm << "\t|du| = " << alpha * deltaUnorm << "\n";
      std::cerr << "\tMerit: " << performanceNew.merit << "\t DynamicsISE: " << performanceNew.stateEqConstraintISE
                << "\t StateInputISE: " << performanceNew.stateInputEqConstraintISE
                << "\t IneqISE: " << performanceNew.inequalityConstraintISE << "\t Penalty: " << performanceNew.inequalityConstraintPenalty
                << "\n";
    }

    // Exit conditions
    bool stepSizeBelowTol = alpha * deltaUnorm < settings_.deltaTol && alpha * deltaXnorm < settings_.deltaTol;
    // Return if step accepted
    if (stepAccepted) {
      x = std::move(xNew);
      u = std::move(uNew);
      bool improvementBelowTol = std::abs(baseline.merit - performanceNew.merit) < costTol && newConstraintViolation < g_min;
      return stepSizeBelowTol || improvementBelowTol;
    }
    // Return if steps get too small without being accepted
    if (stepSizeBelowTol) {
      if (settings_.printLinesearch) {
        std::cerr << "Stepsize is smaller than provided deltaTol -> converged \n";
      }
      return true;
    }
    // Try smaller step
    alpha *= alpha_decay;
  }

  return true;  // Alpha_min reached and no improvement found -> Converged
}

scalar_array_t MultipleShootingSolver::timeDiscretizationWithEvents(scalar_t initTime, scalar_t finalTime, scalar_t dt,
                                                                    const scalar_array_t& eventTimes, scalar_t eventDelta) {
  /*
  A simple example here illustrates the mission of this function

  Assume:
    eventTimes = {3.25, 3.4, 3.88, 4.02, 4.5}
    initTime = 3.0
    finalTime = 4.0
    user_defined delta_t = 0.1
    eps = eventDelta : time added after an event to take the discretization after the mode transition.

  Then the following variables will be:
    timeDiscretization = {3.0, 3.1, 3.2, 3.25 + eps, 3.35, 3.4 + eps, 3.5, 3.6, 3.7, 3.8, 3.88 + eps, 3.98, 4.0}
  */
  assert(dt > 0);
  assert(finalTime > initTime);
  scalar_array_t timeDiscretization;

  // Initialize
  timeDiscretization.push_back(initTime);
  scalar_t nextEventIdx = lookup::findIndexInTimeArray(eventTimes, initTime);

  // Fill iteratively
  scalar_t nextTime = timeDiscretization.back();
  while (timeDiscretization.back() < finalTime) {
    nextTime = nextTime + dt;
    bool nextTimeIsEvent = false;

    // Check if an event has passed
    if (nextEventIdx < eventTimes.size() && nextTime >= eventTimes[nextEventIdx]) {
      nextTime = eventTimes[nextEventIdx];
      nextTimeIsEvent = true;
      nextEventIdx++;
    }

    // Check if final time has passed
    if (nextTime >= finalTime) {
      nextTime = finalTime;
      nextTimeIsEvent = false;
    }

    scalar_t newTimePoint = nextTimeIsEvent ? nextTime + eventDelta : nextTime;
    if (newTimePoint > timeDiscretization.back() + 0.5 * dt) {  // Minimum spacing hardcoded here to 0.5*dt, TODO: make parameter.
      timeDiscretization.push_back(newTimePoint);
    } else {  // Points are close together -> overwrite the old point
      timeDiscretization.back() = newTimePoint;
    }
  }

  return timeDiscretization;
}

}  // namespace ocs2