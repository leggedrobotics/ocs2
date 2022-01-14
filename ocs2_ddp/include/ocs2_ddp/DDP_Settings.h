/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#pragma once

#include <string>

#include <ocs2_core/Types.h>
#include <ocs2_core/integration/Integrator.h>

#include "ocs2_ddp/search_strategy/StrategySettings.h"

namespace ocs2 {
namespace ddp {

/**
 * @brief The DDP algorithm enum
 * Enum used in selecting either SLQ, ILQR algorithms.
 */
enum class Algorithm { SLQ, ILQR };

/**
 * Get string name of DDP algorithm type
 * @param [in] type: DDP algorithm type enum
 */
std::string toAlgorithmName(Algorithm type);

/**
 * Get DDP algorithm type from string name, useful for reading config file
 * @param [in] name: DDP algorithm name
 */
Algorithm fromAlgorithmName(std::string name);

/**
 * This structure contains the settings for the DDP algorithm.
 */
struct Settings {
  /** It should be either SLQ or ILQR */
  Algorithm algorithm_ = Algorithm::SLQ;

  /** Number of threads used in the multi-threading scheme. */
  size_t nThreads_ = 1;
  /** Priority of threads used in the multi-threading scheme. */
  int threadPriority_ = 99;

  /** Maximum number of iterations of DDP. */
  size_t maxNumIterations_ = 15;
  /** This value determines the termination condition based on the minimum relative changes of the cost. */
  scalar_t minRelCost_ = 1e-3;
  /** This value determines the tolerance of constraint's ISE (Integral of Square Error). */
  scalar_t constraintTolerance_ = 1e-3;

  /** This value determines to display the log output DDP. */
  bool displayInfo_ = false;
  /** This value determines to display the a summary log of DDP. */
  bool displayShortSummary_ = false;
  /** Check the numerical stability of the algorithms for debugging purpose. */
  bool checkNumericalStability_ = true;
  /** Printing rollout trajectory for debugging. */
  bool debugPrintRollout_ = false;
  /** Debugs the cached nominal trajectories. */
  bool debugCaching_ = false;

  /** This value determines the absolute tolerance error for ode solvers. */
  scalar_t absTolODE_ = 1e-9;
  /** This value determines the relative tolerance error for ode solvers. */
  scalar_t relTolODE_ = 1e-6;
  /** This value determines the maximum number of integration points per a second for ode solvers. */
  size_t maxNumStepsPerSecond_ = 10000;
  /** The integration time step for Riccati equation which is used for fixed timestep integration scheme. */
  scalar_t timeStep_ = 1e-2;
  /** The backward pass integrator type: SLQ uses it for solving Riccati equation and ILQR uses it for discretizing LQ approximation. */
  IntegratorType backwardPassIntegratorType_ = IntegratorType::ODE45;

  /** The initial coefficient of the quadratic penalty function in augmented Lagrangian method. It should be greater than one. */
  scalar_t constraintPenaltyInitialValue_ = 2.0;
  /** The rate that the coefficient of the quadratic penalty function in augmented Lagrangian method grows. It should be greater than
   * one. */
  scalar_t constraintPenaltyIncreaseRate_ = 2.0;

  /** If true, terms of the Riccati equation will be precomputed before interpolation in the flow-map */
  bool preComputeRiccatiTerms_ = true;

  /** Use either the optimized control policy (true) or the optimized state-input trajectory (false). */
  bool useFeedbackPolicy_ = false;

  /** The risk sensitivity coefficient for risk aware DDP. */
  scalar_t riskSensitiveCoeff_ = 0.0;

  /** Determines the strategy for solving the subproblem. There are two choices line-search strategy and levenberg_marquardt strategy. */
  search_strategy::Type strategy_ = search_strategy::Type::LINE_SEARCH;
  /** The line-search strategy settings. */
  line_search::Settings lineSearch_;
  /** The levenberg_marquardt strategy settings. */
  levenberg_marquardt::Settings levenbergMarquardt_;

};  // end of DDP_Settings

/**
 * This function loads the "DDP_Settings" variables from a config file. This file contains the settings for the SQL and OCS2 algorithms.
 * Here, we use the INFO format which was created specifically for the property tree library (refer to www.goo.gl/fV3yWA).
 *
 * It has the following format: <br>
 * slq  <br>
 * {  <br>
 *   maxIteration        value    <br>
 *   minLearningRate     value    <br>
 *   maxLearningRate     value    <br>
 *   minRelCost          value    <br>
 *   (and so on for the other fields) <br>
 * }  <br>
 *
 * If a value for a specific field is not defined it will set to the default value defined in "DDP_Settings".
 *
 * @param [in] filename: File name which contains the configuration data.
 * @param [in] fieldName: Field name which contains the configuration data.
 * @param [in] verbose: Flag to determine whether to print out the loaded settings or not (The default is true).
 * @return The DDP settings.
 */
Settings loadSettings(const std::string& filename, const std::string& fieldName = "ddp", bool verbose = true);

}  // namespace ddp
}  // namespace ocs2
