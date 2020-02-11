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

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <iostream>
#include <string>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/misc/LoadData.h>

#include "ocs2_ddp/Strategy_Settings.h"

namespace ocs2 {

/**
 * This structure contains the settings for the DDP algorithm.
 */
struct DDP_Settings {
  /** Number of threads used in the multi-threading scheme. */
  size_t nThreads_ = 1;
  /** Priority of threads used in the multi-threading scheme. */
  int threadPriority_ = 99;

  /** Maximum number of iterations of DDP. */
  size_t maxNumIterations_ = 15;
  /** This value determines the termination condition based on the minimum relative changes of the cost. */
  double minRelCost_ = 1e-3;
  /** This value determines the tolerance of constraint's ISE (Integral of Square Error). */
  double constraintTolerance_ = 1e-3;

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
  double absTolODE_ = 1e-9;
  /** This value determines the relative tolerance error for ode solvers. */
  double relTolODE_ = 1e-6;
  /** This value determines the maximum number of integration points per a second for ode solvers. */
  size_t maxNumStepsPerSecond_ = 10000;
  /** The minimum integration time step */
  double minTimeStep_ = 1e-3;

  /** The initial coefficient of the quadratic penalty function in augmented Lagrangian method. It should be greater than one. */
  double constraintPenaltyInitialValue_ = 2.0;
  /** The rate that the coefficient of the quadratic penalty function in augmented Lagrangian method grows. It should be greater than
   * one. */
  double constraintPenaltyIncreaseRate_ = 2.0;
  /** Scaling factor, \f$\mu\f$,  for the inequality constraints barrier */
  double inequalityConstraintMu_ = 0.0;
  /** Threshold parameter, \f$\delta\f$, where the relaxed log barrier function changes from log to quadratic */
  double inequalityConstraintDelta_ = 1e-6;

  /** If true, terms of the Riccati equation will be precomputed before interpolation in the flow-map */
  bool preComputeRiccatiTerms_ = true;

  /** Use either the optimized control policy (true) or the optimized state-input trajectory (false). */
  bool useFeedbackPolicy_ = false;

  /** The risk sensitivity coefficient for risk aware DDP. */
  double riskSensitiveCoeff_ = 0.0;

  /** Determines the strategy for solving the subproblem. There are two choices line-search strategy and levenberg_marquardt strategy. */
  DDP_Strategy strategy_ = DDP_Strategy::LINE_SEARCH;
  /** The line-search strategy settings. */
  Line_Search lineSearch_;
  /** The levenberg_marquardt strategy settings. */
  Levenberg_Marquardt levenbergMarquardt_;

  /**
   * This function loads the "DDP_Settings" variables from a config file. This file contains the settings for the SQL and OCS2 algorithms.
   * Here, we use the INFO format which was created specifically for the property tree library (refer to www.goo.gl/fV3yWA).
   *
   * It has the following format:	<br>
   * slq	<br>
   * {	<br>
   *   maxIteration        value		<br>
   *   minLearningRate     value		<br>
   *   maxLearningRate     value		<br>
   *   minRelCost          value		<br>
   *   (and so on for the other fields)	<br>
   * }	<br>
   *
   * If a value for a specific field is not defined it will set to the default value defined in "DDP_Settings".
   *
   * @param [in] filename: File name which contains the configuration data.
   * @param [in] fieldName: Field name which contains the configuration data.
   * @param [in] verbose: Flag to determine whether to print out the loaded settings or not (The default is true).
   */
  void loadSettings(const std::string& filename, const std::string& fieldName, bool verbose = true) {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    if (verbose) {
      std::cerr << std::endl << " #### DDP Settings: " << std::endl;
      std::cerr << " #### =============================================================================" << std::endl;
    }

    loadData::loadPtreeValue(pt, nThreads_, fieldName + ".nThreads", verbose);
    loadData::loadPtreeValue(pt, threadPriority_, fieldName + ".threadPriority", verbose);

    loadData::loadPtreeValue(pt, maxNumIterations_, fieldName + ".maxNumIterations", verbose);
    loadData::loadPtreeValue(pt, minRelCost_, fieldName + ".minRelCost", verbose);
    loadData::loadPtreeValue(pt, constraintTolerance_, fieldName + ".constraintTolerance", verbose);

    loadData::loadPtreeValue(pt, displayInfo_, fieldName + ".displayInfo", verbose);
    loadData::loadPtreeValue(pt, displayShortSummary_, fieldName + ".displayShortSummary", verbose);
    loadData::loadPtreeValue(pt, checkNumericalStability_, fieldName + ".checkNumericalStability", verbose);
    loadData::loadPtreeValue(pt, debugPrintRollout_, fieldName + ".debugPrintRollout", verbose);
    loadData::loadPtreeValue(pt, debugCaching_, fieldName + ".debugCaching", verbose);

    loadData::loadPtreeValue(pt, absTolODE_, fieldName + ".AbsTolODE", verbose);
    loadData::loadPtreeValue(pt, relTolODE_, fieldName + ".RelTolODE", verbose);
    loadData::loadPtreeValue(pt, maxNumStepsPerSecond_, fieldName + ".maxNumStepsPerSecond", verbose);
    loadData::loadPtreeValue(pt, minTimeStep_, fieldName + ".minTimeStep", verbose);

    loadData::loadPtreeValue(pt, constraintPenaltyInitialValue_, fieldName + ".constraintPenaltyInitialValue", verbose);
    loadData::loadPtreeValue(pt, constraintPenaltyIncreaseRate_, fieldName + ".constraintPenaltyIncreaseRate", verbose);
    loadData::loadPtreeValue(pt, inequalityConstraintMu_, fieldName + ".inequalityConstraintMu", verbose);
    loadData::loadPtreeValue(pt, inequalityConstraintDelta_, fieldName + ".inequalityConstraintDelta", verbose);

    loadData::loadPtreeValue(pt, preComputeRiccatiTerms_, fieldName + ".preComputeRiccatiTerms", verbose);

    loadData::loadPtreeValue(pt, useFeedbackPolicy_, fieldName + ".useFeedbackPolicy", verbose);

    loadData::loadPtreeValue(pt, riskSensitiveCoeff_, fieldName + ".riskSensitiveCoeff", verbose);

    std::string strategyName = ddp_strategy::toString(strategy_);
    loadData::loadPtreeValue(pt, strategyName, fieldName + ".strategy", verbose);
    strategy_ = ddp_strategy::fromString(strategyName);

    switch (strategy_) {
      case DDP_Strategy::LINE_SEARCH: {
        lineSearch_.loadSettings(filename, fieldName + ".lineSearch", verbose);
        break;
      }
      case DDP_Strategy::LEVENBERG_MARQUARDT: {
        levenbergMarquardt_.loadSettings(filename, fieldName + ".levenbergMarquardt", verbose);
        break;
      }
    }

    if (verbose) {
      std::cerr << " #### =============================================================================" << std::endl;
    }
  }

};  // end of DDP_Settings

}  // namespace ocs2
