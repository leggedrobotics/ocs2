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

namespace ocs2 {

/**
 * This structure contains the settings for the DDP algorithm.
 */
class DDP_Settings {
 public:
  using RICCATI_INTEGRATOR_TYPE = Dimensions<0, 0>::RiccatiIntegratorType;

  /**
   * Default constructor.
   */
  DDP_Settings()
      : maxNumIterations_(15),
        minLearningRate_(0.05),
        maxLearningRate_(1.0),
        lineSearchContractionRate_(0.5),
        minRelCost_(1e-3),
        stateConstraintPenaltyCoeff_(0.0),
        stateConstraintPenaltyBase_(1.0),
        inequalityConstraintMu_(0.0),
        inequalityConstraintDelta_(1e-6),
        meritFunctionRho_(1.0),
        constraintStepSize_(1.0),
        displayInfo_(false),
        displayShortSummary_(false)

        ,
        absTolODE_(1e-9),
        relTolODE_(1e-6),
        maxNumStepsPerSecond_(5000),
        minTimeStep_(1e-3),
        minAbsConstraint1ISE_(1e-3),
        minRelConstraint1ISE_(1e-3)

        ,
        simulationIsConstrained_(false),
        noStateConstraints_(false),
        useMakePSD_(true),
        addedRiccatiDiagonal_(1e-5)

        ,
        useMultiThreading_(false),
        nThreads_(4),
        threadPriority_(99),
        debugPrintMT_(false),
        lsStepsizeGreedy_(true),
        checkNumericalStability_(true),
        useRiccatiSolver_(true)

        ,
        useFeedbackPolicy_(false)

        ,
        debugPrintRollout_(false),
        debugCaching_(false)

  {}

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
  void loadSettings(const std::string& filename, const std::string& fieldName, bool verbose = true);

  /**
   * Helper function for loading setting fields
   *
   * @param [in] pt: Property_tree.
   * @param [in] prefix: Field name prefix.
   * @param [in] fieldName: Field name.
   * @param [in] verbose: Print loaded settings if true.
   */
  template <typename T>
  void loadField(const boost::property_tree::ptree& pt, const std::string& prefix, const std::string& fieldName, T& field, bool verbose);

 public:
  /****************
   *** Variables **
   ****************/

  /** Maximum number of iterations of DDP. */
  size_t maxNumIterations_;
  /** Minimum number of iterations of DDP. */
  double minLearningRate_;
  /** Maximum learning rate of line-search scheme in DDP. */
  double maxLearningRate_;
  /** Line-search scheme contraction rate. */
  double lineSearchContractionRate_;
  /** This value determines the termination condition based on the minimum relative changes of the cost. */
  double minRelCost_;
  /** The penalty function coefficient, \f$\alpha\f$, for state-only constraints. \f$ p(i) = \alpha a^i \f$ */
  double stateConstraintPenaltyCoeff_;
  /** The penalty function base, \f$ a \f$, for state-only constraints. \f$ p(i) = \alpha a^i \f$ */
  double stateConstraintPenaltyBase_;
  /** Scaling factor, \f$\mu\f$,  for the inequality constraints barrier */
  double inequalityConstraintMu_;
  /** Threshold parameter, \f$\delta\f$, where the relaxed log barrier function changes from log to quadratic */
  double inequalityConstraintDelta_;
  /** merit function coefficient. */
  double meritFunctionRho_;
  /** Constant step size for type-1 constraints. */
  double constraintStepSize_;
  /** This value determines to display the log output DDP. */
  bool displayInfo_;
  /** This value determines to display the a summary log of DDP. */
  bool displayShortSummary_;

  /** This value determines the absolute tolerance error for ode solvers. */
  double absTolODE_;
  /** This value determines the relative tolerance error for ode solvers. */
  double relTolODE_;
  /** This value determines the maximum number of integration points per a second for ode solvers. */
  size_t maxNumStepsPerSecond_;
  /** The minimum integration time step */
  double minTimeStep_;
  /** This value determines the maximum permitted absolute ISE (Integral of Square Error) for constrained type-1.*/
  double minAbsConstraint1ISE_;
  /** This value determines the maximum permitted relative ISE (Integral of Square Error) for constrained type-1.*/
  double minRelConstraint1ISE_;

  /** Skips calculation of the error correction term (Sve) if the constrained simulation is used for forward simulation.*/
  bool simulationIsConstrained_;

  /** Set true, if a problem does not have state-only constraints. This significantly decreases the runtime of the algorithm. */
  bool noStateConstraints_;

  /** If true DDP makes sure that PSD matrices remain PSD which increases the numerical stability at the expense of extra computation.*/
  bool useMakePSD_;
  /** Add diagonal term to Riccati backward pass for numerical stability. This process is only used when useMakePSD_ set to false.*/
  double addedRiccatiDiagonal_;

  /** Check the numerical stability of the algorithms for debugging purpose. */
  bool checkNumericalStability_;

  /** Use multi threading for the DDP algorithms. */
  bool useMultiThreading_;
  /** Number of threads used in the multi-threading scheme. */
  size_t nThreads_;
  /** Priority of threads used in the multi-threading scheme. */
  int threadPriority_;
  /** Special debugging output for multi-threading scheme. */
  bool debugPrintMT_;
  /**
   * line search options in multi-threading scheme.
   * - True: The largest acceptable step-size will be chosen. This strategy is equivalent to the single core one.
   * - False: The first acceptable step-size will be chosen.
   * */
  bool lsStepsizeGreedy_;

  /** If true, DDP uses ode solver to solve the Riccati equations. Otherwise it uses matrix exponential to solve it. */
  bool useRiccatiSolver_;

  /** Use either the optimized control policy (true) or the optimized state-input trajectory (false). */
  bool useFeedbackPolicy_;

  /** Printing rollout trajectory for debugging. */
  bool debugPrintRollout_;
  /** Debugs the cached nominal trajectories. */
  bool debugCaching_;

};  // end of DDP_Settings class

template <typename T>
inline void DDP_Settings::loadField(const boost::property_tree::ptree& pt, const std::string& prefix, const std::string& fieldName,
                                    T& field, bool verbose) {
  std::string comment;
  try {
    field = pt.get<T>(prefix + "." + fieldName);
  } catch (const std::exception& e) {
    comment = "   \t(default)";
  }

  if (verbose) {
    int fill = std::max<int>(0, 36 - static_cast<int>(fieldName.length()));
    std::cerr << " #### Option loader : option '" << fieldName << "' " << std::string(fill, '.') << " " << field << comment << std::endl;
  }
}

inline void DDP_Settings::loadSettings(const std::string& filename, const std::string& fieldName, bool verbose /*= true*/) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  if (verbose) {
    std::cerr << std::endl << " #### DDP Settings: " << std::endl;
    std::cerr << " #### =============================================================================" << std::endl;
  }

  loadField(pt, fieldName, "useMultiThreading", useMultiThreading_, verbose);
  loadField(pt, fieldName, "nThreads", nThreads_, verbose);
  loadField(pt, fieldName, "threadPriority", threadPriority_, verbose);
  loadField(pt, fieldName, "maxNumIterations", maxNumIterations_, verbose);
  loadField(pt, fieldName, "minLearningRate", minLearningRate_, verbose);
  loadField(pt, fieldName, "maxLearningRate", maxLearningRate_, verbose);
  loadField(pt, fieldName, "minRelCost", minRelCost_, verbose);
  loadField(pt, fieldName, "stateConstraintPenaltyCoeff", stateConstraintPenaltyCoeff_, verbose);
  loadField(pt, fieldName, "stateConstraintPenaltyBase", stateConstraintPenaltyBase_, verbose);
  loadField(pt, fieldName, "inequalityConstraintMu", inequalityConstraintMu_, verbose);
  loadField(pt, fieldName, "inequalityConstraintDelta", inequalityConstraintDelta_, verbose);
  loadField(pt, fieldName, "meritFunctionRho", meritFunctionRho_, verbose);
  loadField(pt, fieldName, "constraintStepSize", constraintStepSize_, verbose);
  loadField(pt, fieldName, "displayInfo", displayInfo_, verbose);
  loadField(pt, fieldName, "displayShortSummary", displayShortSummary_, verbose);
  loadField(pt, fieldName, "AbsTolODE", absTolODE_, verbose);
  loadField(pt, fieldName, "RelTolODE", relTolODE_, verbose);
  loadField(pt, fieldName, "maxNumStepsPerSecond", maxNumStepsPerSecond_, verbose);
  loadField(pt, fieldName, "minTimeStep", minTimeStep_, verbose);
  loadField(pt, fieldName, "simulationIsConstrained", simulationIsConstrained_, verbose);
  loadField(pt, fieldName, "noStateConstraints", noStateConstraints_, verbose);
  loadField(pt, fieldName, "useMakePSD", useMakePSD_, verbose);
  loadField(pt, fieldName, "addedRiccatiDiagonal", addedRiccatiDiagonal_, verbose);
  loadField(pt, fieldName, "minAbsConstraint1ISE", minAbsConstraint1ISE_, verbose);
  loadField(pt, fieldName, "minRelConstraint1ISE", minRelConstraint1ISE_, verbose);
  loadField(pt, fieldName, "debugPrintMT", debugPrintMT_, verbose);
  loadField(pt, fieldName, "lsStepsizeGreedy", lsStepsizeGreedy_, verbose);
  loadField(pt, fieldName, "checkNumericalStability", checkNumericalStability_, verbose);
  loadField(pt, fieldName, "useRiccatiSolver", useRiccatiSolver_, verbose);
  loadField(pt, fieldName, "useFeedbackPolicy", useFeedbackPolicy_, verbose);
  loadField(pt, fieldName, "debugPrintRollout", debugPrintRollout_, verbose);
  loadField(pt, fieldName, "debugCaching", debugCaching_, verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
  }
}

}  // namespace ocs2
