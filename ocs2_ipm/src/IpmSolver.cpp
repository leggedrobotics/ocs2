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

#include "ocs2_ipm/IpmSolver.h"

#include <iomanip>
#include <iostream>
#include <numeric>

#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>
#include <ocs2_oc/multiple_shooting/Helpers.h>
#include <ocs2_oc/multiple_shooting/Initialization.h>
#include <ocs2_oc/multiple_shooting/LagrangianEvaluation.h>
#include <ocs2_oc/multiple_shooting/MetricsComputation.h>
#include <ocs2_oc/multiple_shooting/PerformanceIndexComputation.h>
#include <ocs2_oc/multiple_shooting/Transcription.h>
#include <ocs2_oc/oc_problem/OcpSize.h>

#include "ocs2_ipm/IpmHelpers.h"
#include "ocs2_ipm/IpmInitialization.h"
#include "ocs2_ipm/IpmPerformanceIndexComputation.h"

namespace ocs2 {

namespace {
ipm::Settings rectifySettings(const OptimalControlProblem& ocp, ipm::Settings&& settings) {
  // We have to create the value function if we want to compute the Lagrange multipliers.
  if (settings.computeLagrangeMultipliers) {
    settings.createValueFunction = true;
  }
  // True does not make sense if there are no constraints.
  if (ocp.equalityConstraintPtr->empty()) {
    settings.projectStateInputEqualityConstraints = false;
  }
  // Turn off the barrier update strategy if there are no inequality constraints.
  if (ocp.inequalityConstraintPtr->empty() && ocp.stateInequalityConstraintPtr->empty() && ocp.preJumpInequalityConstraintPtr->empty() &&
      ocp.finalInequalityConstraintPtr->empty()) {
    settings.targetBarrierParameter = settings.initialBarrierParameter;
  }
  return settings;
}
}  // anonymous namespace

IpmSolver::IpmSolver(ipm::Settings settings, const OptimalControlProblem& optimalControlProblem, const Initializer& initializer)
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

IpmSolver::~IpmSolver() {
  if (settings_.printSolverStatistics) {
    std::cerr << getBenchmarkingInformation() << std::endl;
  }
}

void IpmSolver::reset() {
  // Clear solution
  primalSolution_ = PrimalSolution();
  costateTrajectory_.clear();
  projectionMultiplierTrajectory_.clear();
  slackStateIneqTrajectory_.clear();
  dualStateIneqTrajectory_.clear();
  slackStateInputIneqTrajectory_.clear();
  dualStateInputIneqTrajectory_.clear();
  valueFunction_.clear();
  performanceIndeces_.clear();

  // reset timers
  totalNumIterations_ = 0;
  linearQuadraticApproximationTimer_.reset();
  solveQpTimer_.reset();
  linesearchTimer_.reset();
  computeControllerTimer_.reset();
}

std::string IpmSolver::getBenchmarkingInformation() const {
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
    infoStream << "IPM Benchmarking\t   :\tAverage time [ms]   (% of total runtime)\n";
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

const std::vector<PerformanceIndex>& IpmSolver::getIterationsLog() const {
  if (performanceIndeces_.empty()) {
    throw std::runtime_error("[IpmSolver]: No performance log yet, no problem solved yet?");
  } else {
    return performanceIndeces_;
  }
}

ScalarFunctionQuadraticApproximation IpmSolver::getValueFunction(scalar_t time, const vector_t& state) const {
  if (valueFunction_.empty()) {
    throw std::runtime_error("[IpmSolver] Value function is empty! Is createValueFunction true and did the solver run?");
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

void IpmSolver::runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime) {
  if (settings_.printSolverStatus || settings_.printLinesearch) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n+++++++++++++ IPM solver is initialized ++++++++++++++";
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

  // Initialize the costate and projection multiplier
  vector_array_t lmd, nu;
  if (settings_.computeLagrangeMultipliers) {
    initializeCostateTrajectory(timeDiscretization, x, lmd);
    initializeProjectionMultiplierTrajectory(timeDiscretization, nu);
  }

  // Interior point variables
  vector_array_t slackStateIneq, slackStateInputIneq, dualStateIneq, dualStateInputIneq;

  scalar_t barrierParam = settings_.initialBarrierParameter;

  // Bookkeeping
  performanceIndeces_.clear();
  std::vector<Metrics> metrics;

  int iter = 0;
  ipm::Convergence convergence = ipm::Convergence::FALSE;
  while (convergence == ipm::Convergence::FALSE) {
    if (settings_.printSolverStatus || settings_.printLinesearch) {
      std::cerr << "\nIPM iteration: " << iter << " (barrier parameter: " << barrierParam << ")\n";
    }

    // Make QP approximation
    linearQuadraticApproximationTimer_.startTimer();
    const bool initializeSlackAndDualVariables = (iter == 0);
    const auto baselinePerformance =
        setupQuadraticSubproblem(timeDiscretization, initState, x, u, lmd, nu, barrierParam, slackStateIneq, slackStateInputIneq,
                                 dualStateIneq, dualStateInputIneq, initializeSlackAndDualVariables, metrics);
    linearQuadraticApproximationTimer_.endTimer();

    // Solve QP
    solveQpTimer_.startTimer();
    const vector_t delta_x0 = initState - x[0];
    const auto deltaSolution =
        getOCPSolution(delta_x0, barrierParam, slackStateIneq, slackStateInputIneq, dualStateIneq, dualStateInputIneq);
    vector_array_t deltaLmdSol;
    extractValueFunction(timeDiscretization, x, lmd, deltaSolution.deltaXSol, deltaLmdSol);
    solveQpTimer_.endTimer();

    // Apply step
    linesearchTimer_.startTimer();
    const scalar_t maxPrimalStepSize = settings_.usePrimalStepSizeForDual
                                           ? std::min(deltaSolution.maxDualStepSize, deltaSolution.maxPrimalStepSize)
                                           : deltaSolution.maxPrimalStepSize;
    const auto stepInfo = takeStep(baselinePerformance, timeDiscretization, initState, deltaSolution, x, u, barrierParam, slackStateIneq,
                                   slackStateInputIneq, metrics);
    const scalar_t dualStepSize =
        settings_.usePrimalStepSizeForDual ? std::min(stepInfo.stepSize, deltaSolution.maxDualStepSize) : deltaSolution.maxDualStepSize;
    performanceIndeces_.push_back(stepInfo.performanceAfterStep);
    if (settings_.computeLagrangeMultipliers) {
      multiple_shooting::incrementTrajectory(lmd, deltaLmdSol, stepInfo.stepSize, lmd);
      if (settings_.projectStateInputEqualityConstraints) {
        // Remaining term that depends on the costate
        auto deltaNuSol = std::move(deltaSolution.deltaNuSol);
        for (int i = 0; i < nu.size(); i++) {
          if (nu[i].size() > 0) {
            deltaNuSol[i].noalias() += projectionMultiplierCoefficients_[i].dfdcostate * deltaLmdSol[i + 1];
          }
        }
        multiple_shooting::incrementTrajectory(nu, deltaNuSol, stepInfo.stepSize, nu);
      }
    }
    multiple_shooting::incrementTrajectory(dualStateIneq, deltaSolution.deltaDualStateIneq, stepInfo.stepSize, dualStateIneq);
    multiple_shooting::incrementTrajectory(dualStateInputIneq, deltaSolution.deltaDualStateInputIneq, stepInfo.stepSize,
                                           dualStateInputIneq);
    linesearchTimer_.endTimer();

    // Check convergence
    convergence = checkConvergence(iter, barrierParam, baselinePerformance, stepInfo);

    // Update the barrier parameter
    barrierParam = updateBarrierParameter(barrierParam, baselinePerformance, stepInfo);

    // Next iteration
    ++iter;
    ++totalNumIterations_;
  }

  computeControllerTimer_.startTimer();
  primalSolution_ = toPrimalSolution(timeDiscretization, std::move(x), std::move(u));
  costateTrajectory_ = std::move(lmd);
  projectionMultiplierTrajectory_ = std::move(nu);
  slackStateIneqTrajectory_ = std::move(slackStateIneq);
  dualStateIneqTrajectory_ = std::move(dualStateIneq);
  slackStateInputIneqTrajectory_ = std::move(slackStateInputIneq);
  dualStateInputIneqTrajectory_ = std::move(dualStateInputIneq);
  problemMetrics_ = multiple_shooting::toProblemMetrics(timeDiscretization, std::move(metrics));
  computeControllerTimer_.endTimer();

  if (settings_.printSolverStatus || settings_.printLinesearch) {
    std::cerr << "\nConvergence : " << toString(convergence) << "\n";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n+++++++++++++ IPM solver has terminated ++++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
  }
}

void IpmSolver::runParallel(std::function<void(int)> taskFunction) {
  threadPool_.runParallel(std::move(taskFunction), settings_.nThreads);
}

void IpmSolver::initializeCostateTrajectory(const std::vector<AnnotatedTime>& timeDiscretization, const vector_array_t& stateTrajectory,
                                            vector_array_t& costateTrajectory) const {
  const size_t N = static_cast<int>(timeDiscretization.size()) - 1;  // size of the input trajectory
  costateTrajectory.clear();
  costateTrajectory.reserve(N + 1);

  // Determine till when to use the previous solution
  const auto interpolateTill =
      primalSolution_.timeTrajectory_.size() < 2 ? timeDiscretization.front().time : primalSolution_.timeTrajectory_.back();

  const scalar_t initTime = getIntervalStart(timeDiscretization[0]);
  if (initTime < interpolateTill) {
    costateTrajectory.push_back(LinearInterpolation::interpolate(initTime, primalSolution_.timeTrajectory_, costateTrajectory_));
  } else {
    costateTrajectory.push_back(vector_t::Zero(stateTrajectory[0].size()));
  }

  for (int i = 0; i < N; i++) {
    const scalar_t nextTime = getIntervalEnd(timeDiscretization[i + 1]);
    if (nextTime < interpolateTill) {  // interpolate previous solution
      costateTrajectory.push_back(LinearInterpolation::interpolate(nextTime, primalSolution_.timeTrajectory_, costateTrajectory_));
    } else {  // Initialize with zero
      costateTrajectory.push_back(vector_t::Zero(stateTrajectory[i + 1].size()));
    }
  }
}

void IpmSolver::initializeProjectionMultiplierTrajectory(const std::vector<AnnotatedTime>& timeDiscretization,
                                                         vector_array_t& projectionMultiplierTrajectory) const {
  const size_t N = static_cast<int>(timeDiscretization.size()) - 1;  // size of the input trajectory
  projectionMultiplierTrajectory.clear();
  projectionMultiplierTrajectory.reserve(N);
  const auto& ocpDefinition = ocpDefinitions_[0];

  // Determine till when to use the previous solution
  const auto interpolateTill =
      primalSolution_.timeTrajectory_.size() < 2 ? timeDiscretization.front().time : *std::prev(primalSolution_.timeTrajectory_.end(), 2);

  // @todo Fix this using trajectory spreading
  auto interpolateProjectionMultiplierTrajectory = [&](scalar_t time) -> vector_t {
    const size_t numConstraints = ocpDefinition.equalityConstraintPtr->getNumConstraints(time);
    const size_t index = LinearInterpolation::timeSegment(time, primalSolution_.timeTrajectory_).first;
    if (projectionMultiplierTrajectory_.size() > index + 1) {
      if (projectionMultiplierTrajectory_[index].size() == numConstraints &&
          projectionMultiplierTrajectory_[index].size() == projectionMultiplierTrajectory_[index + 1].size()) {
        return LinearInterpolation::interpolate(time, primalSolution_.timeTrajectory_, projectionMultiplierTrajectory_);
      }
    }
    if (projectionMultiplierTrajectory_.size() > index) {
      if (projectionMultiplierTrajectory_[index].size() == numConstraints) {
        return projectionMultiplierTrajectory_[index];
      }
    }
    return vector_t::Zero(numConstraints);
  };

  for (int i = 0; i < N; i++) {
    if (timeDiscretization[i].event == AnnotatedTime::Event::PreEvent) {
      // Event Node
      projectionMultiplierTrajectory.push_back(vector_t());  // no input at event node
    } else {
      // Intermediate node
      const scalar_t time = getIntervalStart(timeDiscretization[i]);
      const size_t numConstraints = ocpDefinition.equalityConstraintPtr->getNumConstraints(time);
      if (time < interpolateTill) {  // interpolate previous solution
        projectionMultiplierTrajectory.push_back(interpolateProjectionMultiplierTrajectory(time));
      } else {  // Initialize with zero
        projectionMultiplierTrajectory.push_back(vector_t::Zero(numConstraints));
      }
    }
  }
}

IpmSolver::OcpSubproblemSolution IpmSolver::getOCPSolution(const vector_t& delta_x0, scalar_t barrierParam,
                                                           const vector_array_t& slackStateIneq, const vector_array_t& slackStateInputIneq,
                                                           const vector_array_t& dualStateIneq, const vector_array_t& dualStateInputIneq) {
  // Solve the QP
  OcpSubproblemSolution solution;
  auto& deltaXSol = solution.deltaXSol;
  auto& deltaUSol = solution.deltaUSol;
  hpipm_status status;
  if (!settings_.projectStateInputEqualityConstraints) {
    hpipmInterface_.resize(extractSizesFromProblem(dynamics_, lagrangian_, &stateInputEqConstraints_));
    status = hpipmInterface_.solve(delta_x0, dynamics_, lagrangian_, &stateInputEqConstraints_, deltaXSol, deltaUSol,
                                   settings_.printSolverStatus);
  } else {  // without constraints, or when using projection, we have an unconstrained QP.
    hpipmInterface_.resize(extractSizesFromProblem(dynamics_, lagrangian_, nullptr));
    status = hpipmInterface_.solve(delta_x0, dynamics_, lagrangian_, nullptr, deltaXSol, deltaUSol, settings_.printSolverStatus);
  }

  if (status != hpipm_status::SUCCESS) {
    throw std::runtime_error("[IpmSolver] Failed to solve QP");
  }

  // to determine if the solution is a descent direction for the cost: compute gradient(cost)' * [dx; du]
  solution.armijoDescentMetric = armijoDescentMetric(lagrangian_, deltaXSol, deltaUSol);

  // Problem horizon
  const int N = static_cast<int>(deltaXSol.size()) - 1;

  auto& deltaNuSol = solution.deltaNuSol;
  auto& deltaSlackStateIneq = solution.deltaSlackStateIneq;
  auto& deltaSlackStateInputIneq = solution.deltaSlackStateInputIneq;
  auto& deltaDualStateIneq = solution.deltaDualStateIneq;
  auto& deltaDualStateInputIneq = solution.deltaDualStateInputIneq;
  deltaNuSol.resize(N);
  deltaSlackStateIneq.resize(N + 1);
  deltaSlackStateInputIneq.resize(N + 1);
  deltaDualStateIneq.resize(N + 1);
  deltaDualStateInputIneq.resize(N + 1);

  scalar_array_t primalStepSizes(settings_.nThreads, 1.0);
  scalar_array_t dualStepSizes(settings_.nThreads, 1.0);

  std::atomic_int timeIndex{0};
  auto parallelTask = [&](int workerId) {
    // Get worker specific resources
    vector_t tmp;  // 1 temporary for re-use for projection.

    int i = timeIndex++;
    while (i < N) {
      deltaSlackStateIneq[i] = ipm::retrieveSlackDirection(stateIneqConstraints_[i], deltaXSol[i], barrierParam, slackStateIneq[i]);
      deltaDualStateIneq[i] = ipm::retrieveDualDirection(barrierParam, slackStateIneq[i], dualStateIneq[i], deltaSlackStateIneq[i]);
      deltaSlackStateInputIneq[i] =
          ipm::retrieveSlackDirection(stateInputIneqConstraints_[i], deltaXSol[i], deltaUSol[i], barrierParam, slackStateInputIneq[i]);
      deltaDualStateInputIneq[i] =
          ipm::retrieveDualDirection(barrierParam, slackStateInputIneq[i], dualStateInputIneq[i], deltaSlackStateInputIneq[i]);
      primalStepSizes[workerId] = std::min(
          {primalStepSizes[workerId],
           ipm::fractionToBoundaryStepSize(slackStateIneq[i], deltaSlackStateIneq[i], settings_.fractionToBoundaryMargin),
           ipm::fractionToBoundaryStepSize(slackStateInputIneq[i], deltaSlackStateInputIneq[i], settings_.fractionToBoundaryMargin)});
      dualStepSizes[workerId] = std::min(
          {dualStepSizes[workerId],
           ipm::fractionToBoundaryStepSize(dualStateIneq[i], deltaDualStateIneq[i], settings_.fractionToBoundaryMargin),
           ipm::fractionToBoundaryStepSize(dualStateInputIneq[i], deltaDualStateInputIneq[i], settings_.fractionToBoundaryMargin)});
      if (constraintsProjection_[i].f.size() > 0) {
        if (settings_.computeLagrangeMultipliers) {
          deltaNuSol[i] = projectionMultiplierCoefficients_[i].f;
          deltaNuSol[i].noalias() += projectionMultiplierCoefficients_[i].dfdx * deltaXSol[i];
          deltaNuSol[i].noalias() += projectionMultiplierCoefficients_[i].dfdu * deltaUSol[i];
        }
        // Re-map the projected input back to the original space.
        tmp.noalias() = constraintsProjection_[i].dfdu * deltaUSol[i];
        deltaUSol[i] = tmp + constraintsProjection_[i].f;
        deltaUSol[i].noalias() += constraintsProjection_[i].dfdx * deltaXSol[i];
      }

      i = timeIndex++;
    }

    if (i == N) {  // Only one worker will execute this
      deltaSlackStateIneq[i] = ipm::retrieveSlackDirection(stateIneqConstraints_[i], deltaXSol[i], barrierParam, slackStateIneq[i]);
      deltaDualStateIneq[i] = ipm::retrieveDualDirection(barrierParam, slackStateIneq[i], dualStateIneq[i], deltaSlackStateIneq[i]);
      primalStepSizes[workerId] =
          std::min(primalStepSizes[workerId],
                   ipm::fractionToBoundaryStepSize(slackStateIneq[i], deltaSlackStateIneq[i], settings_.fractionToBoundaryMargin));
      dualStepSizes[workerId] = std::min(dualStepSizes[workerId], ipm::fractionToBoundaryStepSize(dualStateIneq[i], deltaDualStateIneq[i],
                                                                                                  settings_.fractionToBoundaryMargin));
    }
  };
  runParallel(std::move(parallelTask));

  solution.maxPrimalStepSize = *std::min_element(primalStepSizes.begin(), primalStepSizes.end());
  solution.maxDualStepSize = *std::min_element(dualStepSizes.begin(), dualStepSizes.end());

  return solution;
}

void IpmSolver::extractValueFunction(const std::vector<AnnotatedTime>& time, const vector_array_t& x, const vector_array_t& lmd,
                                     const vector_array_t& deltaXSol, vector_array_t& deltaLmdSol) {
  if (settings_.createValueFunction) {
    valueFunction_ = hpipmInterface_.getRiccatiCostToGo(dynamics_[0], lagrangian_[0]);
    // Compute costate directions
    deltaLmdSol.resize(deltaXSol.size());
    for (int i = 0; i < time.size(); ++i) {
      deltaLmdSol[i] = valueFunction_[i].dfdx;
      deltaLmdSol[i].noalias() += valueFunction_[i].dfdxx * x[i];
    }
    // Correct for linearization state
    for (int i = 0; i < time.size(); ++i) {
      valueFunction_[i].dfdx.noalias() -= valueFunction_[i].dfdxx * x[i];
      if (settings_.computeLagrangeMultipliers) {
        valueFunction_[i].dfdx.noalias() += lmd[i];
      }
    }
  }
}

PrimalSolution IpmSolver::toPrimalSolution(const std::vector<AnnotatedTime>& time, vector_array_t&& x, vector_array_t&& u) {
  if (settings_.useFeedbackPolicy) {
    ModeSchedule modeSchedule = this->getReferenceManager().getModeSchedule();
    matrix_array_t KMatrices = hpipmInterface_.getRiccatiFeedback(dynamics_[0], lagrangian_[0]);
    if (settings_.projectStateInputEqualityConstraints) {
      multiple_shooting::remapProjectedGain(constraintsProjection_, KMatrices);
    }
    return multiple_shooting::toPrimalSolution(time, std::move(modeSchedule), std::move(x), std::move(u), std::move(KMatrices));

  } else {
    ModeSchedule modeSchedule = this->getReferenceManager().getModeSchedule();
    return multiple_shooting::toPrimalSolution(time, std::move(modeSchedule), std::move(x), std::move(u));
  }
}

PerformanceIndex IpmSolver::setupQuadraticSubproblem(const std::vector<AnnotatedTime>& time, const vector_t& initState,
                                                     const vector_array_t& x, const vector_array_t& u, const vector_array_t& lmd,
                                                     const vector_array_t& nu, scalar_t barrierParam, vector_array_t& slackStateIneq,
                                                     vector_array_t& slackStateInputIneq, vector_array_t& dualStateIneq,
                                                     vector_array_t& dualStateInputIneq, bool initializeSlackAndDualVariables,
                                                     std::vector<Metrics>& metrics) {
  // Problem horizon
  const int N = static_cast<int>(time.size()) - 1;

  std::vector<PerformanceIndex> performance(settings_.nThreads, PerformanceIndex());
  lagrangian_.resize(N + 1);
  dynamics_.resize(N);
  stateInputEqConstraints_.resize(N + 1);
  stateIneqConstraints_.resize(N + 1);
  stateInputIneqConstraints_.resize(N + 1);
  constraintsProjection_.resize(N);
  projectionMultiplierCoefficients_.resize(N);
  metrics.resize(N + 1);

  if (initializeSlackAndDualVariables) {
    slackStateIneq.resize(N + 1);
    slackStateInputIneq.resize(N + 1);
    dualStateIneq.resize(N + 1);
    dualStateInputIneq.resize(N + 1);
  }

  std::atomic_int timeIndex{0};
  auto parallelTask = [&](int workerId) {
    // Get worker specific resources
    OptimalControlProblem& ocpDefinition = ocpDefinitions_[workerId];

    int i = timeIndex++;
    while (i < N) {
      if (time[i].event == AnnotatedTime::Event::PreEvent) {
        // Event node
        auto result = multiple_shooting::setupEventNode(ocpDefinition, time[i].time, x[i], x[i + 1]);
        if (initializeSlackAndDualVariables) {
          slackStateIneq[i] =
              ipm::initializeSlackVariable(result.ineqConstraints.f, settings_.initialSlackLowerBound, settings_.initialSlackMarginRate);
          dualStateIneq[i] = ipm::initializeDualVariable(slackStateIneq[i], barrierParam, settings_.initialDualLowerBound,
                                                         settings_.initialDualMarginRate);
        }
        metrics[i] = multiple_shooting::computeMetrics(result);
        performance[workerId] += ipm::computePerformanceIndex(result, barrierParam, slackStateIneq[i]);
        dynamics_[i] = std::move(result.dynamics);
        stateInputEqConstraints_[i].resize(0, x[i].size());
        stateIneqConstraints_[i] = std::move(result.ineqConstraints);
        stateInputIneqConstraints_[i].resize(0, x[i].size());
        constraintsProjection_[i].resize(0, x[i].size());
        projectionMultiplierCoefficients_[i] = multiple_shooting::ProjectionMultiplierCoefficients();
        if (settings_.computeLagrangeMultipliers) {
          lagrangian_[i] = multiple_shooting::evaluateLagrangianEventNode(lmd[i], lmd[i + 1], std::move(result.cost), dynamics_[i]);
        } else {
          lagrangian_[i] = std::move(result.cost);
        }

        ipm::condenseIneqConstraints(barrierParam, slackStateIneq[i], dualStateIneq[i], stateIneqConstraints_[i], lagrangian_[i]);
        performance[workerId].dualFeasibilitiesSSE += multiple_shooting::evaluateDualFeasibilities(lagrangian_[i]);
        performance[workerId].dualFeasibilitiesSSE +=
            ipm::evaluateComplementarySlackness(barrierParam, slackStateIneq[i], dualStateIneq[i]);
      } else {
        // Normal, intermediate node
        const scalar_t ti = getIntervalStart(time[i]);
        const scalar_t dt = getIntervalDuration(time[i], time[i + 1]);
        auto result = multiple_shooting::setupIntermediateNode(ocpDefinition, sensitivityDiscretizer_, ti, dt, x[i], x[i + 1], u[i]);
        // Disable the state-only inequality constraints at the initial node
        if (i == 0) {
          result.stateIneqConstraints.setZero(0, x[i].size());
          std::fill(result.constraintsSize.stateIneq.begin(), result.constraintsSize.stateIneq.end(), 0);
        }
        if (initializeSlackAndDualVariables) {
          slackStateIneq[i] = ipm::initializeSlackVariable(result.stateIneqConstraints.f, settings_.initialSlackLowerBound,
                                                           settings_.initialSlackMarginRate);
          slackStateInputIneq[i] = ipm::initializeSlackVariable(result.stateInputIneqConstraints.f, settings_.initialSlackLowerBound,
                                                                settings_.initialSlackMarginRate);
          dualStateIneq[i] = ipm::initializeDualVariable(slackStateIneq[i], barrierParam, settings_.initialDualLowerBound,
                                                         settings_.initialDualMarginRate);
          dualStateInputIneq[i] = ipm::initializeDualVariable(slackStateInputIneq[i], barrierParam, settings_.initialDualLowerBound,
                                                              settings_.initialDualMarginRate);
        }
        metrics[i] = multiple_shooting::computeMetrics(result);
        performance[workerId] += ipm::computePerformanceIndex(result, dt, barrierParam, slackStateIneq[i], slackStateInputIneq[i]);
        if (settings_.projectStateInputEqualityConstraints) {
          multiple_shooting::projectTranscription(result, settings_.computeLagrangeMultipliers);
        }
        dynamics_[i] = std::move(result.dynamics);
        stateInputEqConstraints_[i] = std::move(result.stateInputEqConstraints);
        stateIneqConstraints_[i] = std::move(result.stateIneqConstraints);
        stateInputIneqConstraints_[i] = std::move(result.stateInputIneqConstraints);
        constraintsProjection_[i] = std::move(result.constraintsProjection);
        projectionMultiplierCoefficients_[i] = std::move(result.projectionMultiplierCoefficients);
        if (settings_.computeLagrangeMultipliers) {
          lagrangian_[i] = multiple_shooting::evaluateLagrangianIntermediateNode(lmd[i], lmd[i + 1], nu[i], std::move(result.cost),
                                                                                 dynamics_[i], stateInputEqConstraints_[i]);
        } else {
          lagrangian_[i] = std::move(result.cost);
        }

        ipm::condenseIneqConstraints(barrierParam, slackStateIneq[i], dualStateIneq[i], stateIneqConstraints_[i], lagrangian_[i]);
        ipm::condenseIneqConstraints(barrierParam, slackStateInputIneq[i], dualStateInputIneq[i], stateInputIneqConstraints_[i],
                                     lagrangian_[i]);
        performance[workerId].dualFeasibilitiesSSE += multiple_shooting::evaluateDualFeasibilities(lagrangian_[i]);
        performance[workerId].dualFeasibilitiesSSE +=
            ipm::evaluateComplementarySlackness(barrierParam, slackStateIneq[i], dualStateIneq[i]);
        performance[workerId].dualFeasibilitiesSSE +=
            ipm::evaluateComplementarySlackness(barrierParam, slackStateInputIneq[i], dualStateInputIneq[i]);
      }

      i = timeIndex++;
    }

    if (i == N) {  // Only one worker will execute this
      const scalar_t tN = getIntervalStart(time[N]);
      auto result = multiple_shooting::setupTerminalNode(ocpDefinition, tN, x[N]);
      if (initializeSlackAndDualVariables) {
        slackStateIneq[N] =
            ipm::initializeSlackVariable(result.ineqConstraints.f, settings_.initialSlackLowerBound, settings_.initialSlackMarginRate);
        dualStateIneq[N] =
            ipm::initializeDualVariable(slackStateIneq[N], barrierParam, settings_.initialDualLowerBound, settings_.initialDualMarginRate);
      }
      metrics[i] = multiple_shooting::computeMetrics(result);
      performance[workerId] += ipm::computePerformanceIndex(result, barrierParam, slackStateIneq[N]);
      stateInputEqConstraints_[i].resize(0, x[i].size());
      stateIneqConstraints_[i] = std::move(result.ineqConstraints);
      if (settings_.computeLagrangeMultipliers) {
        lagrangian_[i] = multiple_shooting::evaluateLagrangianTerminalNode(lmd[i], std::move(result.cost));
      } else {
        lagrangian_[i] = std::move(result.cost);
      }
      ipm::condenseIneqConstraints(barrierParam, slackStateIneq[N], dualStateIneq[N], stateIneqConstraints_[N], lagrangian_[N]);
      performance[workerId].dualFeasibilitiesSSE += multiple_shooting::evaluateDualFeasibilities(lagrangian_[N]);
      performance[workerId].dualFeasibilitiesSSE += ipm::evaluateComplementarySlackness(barrierParam, slackStateIneq[N], dualStateIneq[N]);
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

PerformanceIndex IpmSolver::computePerformance(const std::vector<AnnotatedTime>& time, const vector_t& initState, const vector_array_t& x,
                                               const vector_array_t& u, scalar_t barrierParam, const vector_array_t& slackStateIneq,
                                               const vector_array_t& slackStateInputIneq, std::vector<Metrics>& metrics) {
  // Problem horizon
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
        performance[workerId] += ipm::toPerformanceIndex(metrics[i], barrierParam, slackStateIneq[i]);
      } else {
        // Normal, intermediate node
        const scalar_t ti = getIntervalStart(time[i]);
        const scalar_t dt = getIntervalDuration(time[i], time[i + 1]);
        const bool enableStateInequalityConstraints = (i > 0);
        metrics[i] = multiple_shooting::computeIntermediateMetrics(ocpDefinition, discretizer_, ti, dt, x[i], x[i + 1], u[i]);
        performance[workerId] += ipm::toPerformanceIndex(metrics[i], dt, barrierParam, slackStateIneq[i], slackStateInputIneq[i],
                                                         enableStateInequalityConstraints);
      }

      i = timeIndex++;
    }

    if (i == N) {  // Only one worker will execute this
      const scalar_t tN = getIntervalStart(time[N]);
      metrics[N] = multiple_shooting::computeTerminalMetrics(ocpDefinition, tN, x[N]);
      performance[workerId] += ipm::toPerformanceIndex(metrics[N], barrierParam, slackStateIneq[N]);
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

ipm::StepInfo IpmSolver::takeStep(const PerformanceIndex& baseline, const std::vector<AnnotatedTime>& timeDiscretization,
                                  const vector_t& initState, const OcpSubproblemSolution& subproblemSolution, vector_array_t& x,
                                  vector_array_t& u, scalar_t barrierParam, vector_array_t& slackStateIneq,
                                  vector_array_t& slackStateInputIneq, std::vector<Metrics>& metrics) {
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
  const auto& deltaSlackStateIneq = subproblemSolution.deltaSlackStateIneq;
  const auto& deltaSlackStateInputIneq = subproblemSolution.deltaSlackStateInputIneq;
  const auto deltaUnorm = multiple_shooting::trajectoryNorm(du);
  const auto deltaXnorm = multiple_shooting::trajectoryNorm(dx);

  scalar_t alpha = subproblemSolution.maxPrimalStepSize;
  vector_array_t xNew(x.size());
  vector_array_t uNew(u.size());
  vector_array_t slackStateIneqNew(slackStateIneq.size());
  vector_array_t slackStateInputIneqNew(slackStateInputIneq.size());
  std::vector<Metrics> metricsNew(metrics.size());
  do {
    // Compute step
    multiple_shooting::incrementTrajectory(u, du, alpha, uNew);
    multiple_shooting::incrementTrajectory(x, dx, alpha, xNew);
    multiple_shooting::incrementTrajectory(slackStateIneq, deltaSlackStateIneq, alpha, slackStateIneqNew);
    multiple_shooting::incrementTrajectory(slackStateInputIneq, deltaSlackStateInputIneq, alpha, slackStateInputIneqNew);

    // Compute cost and constraints
    const PerformanceIndex performanceNew =
        computePerformance(timeDiscretization, initState, xNew, uNew, barrierParam, slackStateIneqNew, slackStateInputIneqNew, metricsNew);

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
      slackStateIneq = std::move(slackStateIneqNew);
      slackStateInputIneq = std::move(slackStateInputIneqNew);
      metrics = std::move(metricsNew);

      // Prepare step info
      ipm::StepInfo stepInfo;
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
  ipm::StepInfo stepInfo;
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

scalar_t IpmSolver::updateBarrierParameter(scalar_t currentBarrierParameter, const PerformanceIndex& baseline,
                                           const ipm::StepInfo& stepInfo) const {
  if (currentBarrierParameter <= settings_.targetBarrierParameter) {
    return currentBarrierParameter;
  } else if (std::abs(stepInfo.performanceAfterStep.merit - baseline.merit) < settings_.barrierReductionCostTol &&
             FilterLinesearch::totalConstraintViolation(stepInfo.performanceAfterStep) < settings_.barrierReductionConstraintTol) {
    return std::min((currentBarrierParameter * settings_.barrierLinearDecreaseFactor),
                    std::pow(currentBarrierParameter, settings_.barrierSuperlinearDecreasePower));
  } else {
    return currentBarrierParameter;
  }
}

ipm::Convergence IpmSolver::checkConvergence(int iteration, scalar_t barrierParam, const PerformanceIndex& baseline,
                                             const ipm::StepInfo& stepInfo) const {
  using Convergence = ipm::Convergence;
  if ((iteration + 1) >= settings_.ipmIteration) {
    // Converged because the next iteration would exceed the specified number of iterations
    return Convergence::ITERATIONS;
  } else if (stepInfo.stepSize < settings_.alpha_min) {
    // Converged because step size is below the specified minimum
    return Convergence::STEPSIZE;
  } else if (std::abs(stepInfo.performanceAfterStep.merit - baseline.merit) < settings_.costTol &&
             FilterLinesearch::totalConstraintViolation(stepInfo.performanceAfterStep) < settings_.g_min) {
    // Converged because the change in merit is below the specified tolerance while the constraint violation is below the minimum
    return Convergence::METRICS;
  } else if (stepInfo.dx_norm < settings_.deltaTol && stepInfo.du_norm < settings_.deltaTol &&
             barrierParam <= settings_.targetBarrierParameter) {
    // Converged because the change in primal variables is below the specified tolerance
    return Convergence::PRIMAL;
  } else {
    // None of the above convergence criteria were met -> not converged.
    return Convergence::FALSE;
  }
}

}  // namespace ocs2
