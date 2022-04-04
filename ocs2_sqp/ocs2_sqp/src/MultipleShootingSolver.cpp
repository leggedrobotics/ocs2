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

#include "ocs2_sqp/MultipleShootingSolver.h"

#include <iostream>
#include <numeric>

#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/penalties/penalties/RelaxedBarrierPenalty.h>

#include "ocs2_sqp/MultipleShootingInitialization.h"
#include "ocs2_sqp/MultipleShootingTranscription.h"

namespace ocs2 {

MultipleShootingSolver::MultipleShootingSolver(Settings settings, const OptimalControlProblem& optimalControlProblem,
                                               const Initializer& initializer)
    : SolverBase(),
      settings_(std::move(settings)),
      hpipmInterface_(hpipm_interface::OcpSize(), settings.hpipmSettings),
      threadPool_(std::max(settings_.nThreads, size_t(1)) - 1, settings_.threadPriority) {
  Eigen::setNbThreads(1);  // No multithreading within Eigen.
  Eigen::initParallel();

  // Dynamics discretization
  discretizer_ = selectDynamicsDiscretization(settings.integratorType);
  sensitivityDiscretizer_ = selectDynamicsSensitivityDiscretization(settings.integratorType);

  // Clone objects to have one for each worker
  for (int w = 0; w < settings.nThreads; w++) {
    ocpDefinitions_.push_back(optimalControlProblem);
  }

  // Operating points
  initializerPtr_.reset(initializer.clone());

  if (optimalControlProblem.equalityConstraintPtr->empty()) {
    settings_.projectStateInputEqualityConstraints = false;  // True does not make sense if there are no constraints.
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
  numProblems_ = 0;
  totalNumIterations_ = 0;
  linearQuadraticApproximationTimer_.reset();
  solveQpTimer_.reset();
  linesearchTimer_.reset();
  computeControllerTimer_.reset();
}

std::string MultipleShootingSolver::getBenchmarkingInformation() const {
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

const std::vector<PerformanceIndex>& MultipleShootingSolver::getIterationsLog() const {
  if (performanceIndeces_.empty()) {
    throw std::runtime_error("[MultipleShootingSolver]: No performance log yet, no problem solved yet?");
  } else {
    return performanceIndeces_;
  }
}

void MultipleShootingSolver::runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime) {
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

void MultipleShootingSolver::runParallel(std::function<void(int)> taskFunction) {
  threadPool_.runParallel(std::move(taskFunction), settings_.nThreads);
}

void MultipleShootingSolver::initializeStateInputTrajectories(const vector_t& initState,
                                                              const std::vector<AnnotatedTime>& timeDiscretization,
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

MultipleShootingSolver::OcpSubproblemSolution MultipleShootingSolver::getOCPSolution(const vector_t& delta_x0) {
  // Solve the QP
  OcpSubproblemSolution solution;
  auto& deltaXSol = solution.deltaXSol;
  auto& deltaUSol = solution.deltaUSol;
  hpipm_status status;
  const bool hasStateInputConstraints = !ocpDefinitions_.front().equalityConstraintPtr->empty();
  if (hasStateInputConstraints && !settings_.projectStateInputEqualityConstraints) {
    hpipmInterface_.resize(hpipm_interface::extractSizesFromProblem(dynamics_, cost_, &constraints_));
    status = hpipmInterface_.solve(delta_x0, dynamics_, cost_, &constraints_, deltaXSol, deltaUSol, settings_.printSolverStatus);
  } else {  // without constraints, or when using projection, we have an unconstrained QP.
    hpipmInterface_.resize(hpipm_interface::extractSizesFromProblem(dynamics_, cost_, nullptr));
    status = hpipmInterface_.solve(delta_x0, dynamics_, cost_, nullptr, deltaXSol, deltaUSol, settings_.printSolverStatus);
  }

  if (status != hpipm_status::SUCCESS) {
    throw std::runtime_error("[MultipleShootingSolver] Failed to solve QP");
  }

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

void MultipleShootingSolver::setPrimalSolution(const std::vector<AnnotatedTime>& time, vector_array_t&& x, vector_array_t&& u) {
  // Clear old solution
  primalSolution_.clear();

  // Correct for missing inputs at PreEvents
  for (int i = 0; i < time.size(); ++i) {
    if (time[i].event == AnnotatedTime::Event::PreEvent && i > 0) {
      u[i] = u[i - 1];
    }
  }

  // Compute feedback, before x and u are moved to primal solution
  vector_array_t uff;
  matrix_array_t controllerGain;
  if (settings_.useFeedbackPolicy) {
    // see doc/LQR_full.pdf for detailed derivation for feedback terms
    uff = u;  // Copy and adapt in loop
    controllerGain.reserve(time.size());
    matrix_array_t KMatrices = hpipmInterface_.getRiccatiFeedback(dynamics_[0], cost_[0]);
    for (int i = 0; (i + 1) < time.size(); i++) {
      if (time[i].event == AnnotatedTime::Event::PreEvent && i > 0) {
        uff[i] = uff[i - 1];
        controllerGain.push_back(controllerGain.back());
      } else {
        // Linear controller has convention u = uff + K * x;
        // We computed u = u'(t) + K (x - x'(t));
        // >> uff = u'(t) - K x'(t)
        if (constraintsProjection_[i].f.size() > 0) {
          controllerGain.push_back(std::move(constraintsProjection_[i].dfdx));  // Steal! Don't use after this.
          controllerGain.back().noalias() += constraintsProjection_[i].dfdu * KMatrices[i];
        } else {
          controllerGain.push_back(std::move(KMatrices[i]));
        }
        uff[i].noalias() -= controllerGain.back() * x[i];
      }
    }
    // Copy last one to get correct length
    uff.push_back(uff.back());
    controllerGain.push_back(controllerGain.back());
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

  // Assign controller
  if (settings_.useFeedbackPolicy) {
    primalSolution_.controllerPtr_.reset(new LinearController(primalSolution_.timeTrajectory_, std::move(uff), std::move(controllerGain)));
  } else {
    primalSolution_.controllerPtr_.reset(new FeedforwardController(primalSolution_.timeTrajectory_, primalSolution_.inputTrajectory_));
  }
}

PerformanceIndex MultipleShootingSolver::setupQuadraticSubproblem(const std::vector<AnnotatedTime>& time, const vector_t& initState,
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

PerformanceIndex MultipleShootingSolver::computePerformance(const std::vector<AnnotatedTime>& time, const vector_t& initState,
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

scalar_t MultipleShootingSolver::trajectoryNorm(const vector_array_t& v) {
  scalar_t norm = 0.0;
  for (const auto& vi : v) {
    norm += vi.squaredNorm();
  }
  return std::sqrt(norm);
}

scalar_t MultipleShootingSolver::totalConstraintViolation(const PerformanceIndex& performance) const {
  return std::sqrt(performance.dynamicsViolationSSE + performance.equalityConstraintsSSE);
}

multiple_shooting::StepInfo MultipleShootingSolver::takeStep(const PerformanceIndex& baseline,
                                                             const std::vector<AnnotatedTime>& timeDiscretization,
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

multiple_shooting::Convergence MultipleShootingSolver::checkConvergence(int iteration, const PerformanceIndex& baseline,
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
