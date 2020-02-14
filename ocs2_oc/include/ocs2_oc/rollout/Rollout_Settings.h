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

#include <ocs2_core/integration/Integrator.h>
#include <ocs2_core/misc/LoadData.h>

#include "ocs2_oc/rollout/RootFinderType.h"

namespace ocs2 {

/**
 * This structure contains the settings for forward rollout algorithms.
 */
class Rollout_Settings {
 public:
  /**
   * Constructor with all settings as arguments.
   *
   * @param [in] absTolODE: Absolute tolerance of the rollout.
   * @param [in] relTolODE: Relative tolerance of the rollout.
   * @param [in] maxNumStepsPerSecond: Maximum number of steps in the rollout.
   * @param [in] minTimeStep: Minimum time step of the rollout.
   * @param [in] integratorType: Rollout integration scheme type.
   * @param [in] checkNumericalStability: Whether to check that the rollout is numerically stable.
   * @param [in] reconstructInputTrajectory: Whether to run controller again after integration to construct input trajectory
   */
  explicit Rollout_Settings(double absTolODE = 1e-9, double relTolODE = 1e-6, size_t maxNumStepsPerSecond = 5000, double minTimeStep = 1e-3,
                            IntegratorType integratorType = IntegratorType::ODE45, bool checkNumericalStability = false,
                            bool reconstructInputTrajectory = true, RootFinderType rootFindingAlgorithm = RootFinderType::ANDERSON_BJORCK,
                            int maxSingleEventIterations = 10, bool useTrajectorySpreadingController = false)
      : absTolODE_(absTolODE),
        relTolODE_(relTolODE),
        maxNumStepsPerSecond_(maxNumStepsPerSecond),
        minTimeStep_(minTimeStep),
        integratorType_(integratorType),
        checkNumericalStability_(checkNumericalStability),
        reconstructInputTrajectory_(reconstructInputTrajectory),
        rootFindingAlgorithm_(rootFindingAlgorithm),
        maxSingleEventIterations_(maxSingleEventIterations),
        useTrajectorySpreadingController_(useTrajectorySpreadingController) {}

  /**
   * This function loads the "Rollout_Settings" variables from a config file. This file contains the settings for the Rollout algorithms.
   * Here, we use the INFO format which was created specifically for the property tree library (refer to www.goo.gl/fV3yWA).
   *
   * It has the following format:	<br>
   * rollout	<br>
   * {	<br>
   *   absTolODE                value		<br>
   *   relTolODE                value		<br>
   *   maxNumStepsPerSecond     value		<br>
   *   minTimeStep              value		<br>
   *   (and so on for the other fields)	<br>
   * }	<br>
   *
   * If a value for a specific field is not defined it will set to the default value defined in "Rollout_Settings".
   *
   * @param [in] filename: File name which contains the configuration data.
   * @param [in] fieldName: Field name which contains the configuration data.
   * @param [in] verbose: Flag to determine whether to print out the loaded settings or not (The default is true).
   */
  void loadSettings(const std::string& filename, const std::string& fieldName, bool verbose = true);

 public:
  /****************
   *** Variables **
   ****************/
  /** This value determines the absolute tolerance error for ode solvers. */
  double absTolODE_;
  /** This value determines the relative tolerance error for ode solvers. */
  double relTolODE_;
  /** This value determines the maximum number of integration points per a second for ode solvers. */
  size_t maxNumStepsPerSecond_;
  /** The minimum integration time step */
  double minTimeStep_;
  /** Rollout integration scheme type */
  IntegratorType integratorType_;
  /** Whether to check that the rollout is numerically stable */
  bool checkNumericalStability_;
  /** Whether to run controller again after integration to construct input trajectory */
  bool reconstructInputTrajectory_;
  /** Which of the RootFinding algorithms to use in StateRollout
   * 		0:		Anderson & Björck		(default)
   * 		1:		Pegasus
   * 		2:		Illinois
   * 		3:		Regula Falsi
   */
  RootFinderType rootFindingAlgorithm_;
  /** This value determines the maximum number of iterations, per event, allowed in state triggered rollout to find
   *  the guard surface zero crossing.  */
  int maxSingleEventIterations_;
  /** Whether to use the trajectory spreading controller in state triggered rollout */
  bool useTrajectorySpreadingController_;

};  // end of Rollout_Settings class

inline void Rollout_Settings::loadSettings(const std::string& filename, const std::string& fieldName, bool verbose /*= true*/) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  if (verbose) {
    std::cerr << std::endl << " #### Rollout Settings: " << std::endl;
    std::cerr << " #### =============================================================================" << std::endl;
  }

  loadData::loadPtreeValue(pt, absTolODE_, fieldName + ".AbsTolODE", verbose);
  loadData::loadPtreeValue(pt, relTolODE_, fieldName + ".RelTolODE", verbose);
  loadData::loadPtreeValue(pt, maxNumStepsPerSecond_, fieldName + ".maxNumStepsPerSecond", verbose);
  loadData::loadPtreeValue(pt, minTimeStep_, fieldName + ".minTimeStep", verbose);

  auto integratorName = integrator_type::toString(integratorType_);  // keep default
  loadData::loadPtreeValue(pt, integratorName, fieldName + ".integratorType", verbose);
  integratorType_ = integrator_type::fromString(integratorName);

  loadData::loadPtreeValue(pt, checkNumericalStability_, fieldName + ".checkNumericalStability", verbose);
  loadData::loadPtreeValue(pt, reconstructInputTrajectory_, fieldName + ".reconstructInputTrajectory", verbose);

  int tmp = static_cast<int>(rootFindingAlgorithm_);
  loadData::loadPtreeValue(pt, tmp, fieldName + ".rootFindingAlgorithm", verbose);
  rootFindingAlgorithm_ = static_cast<RootFinderType>(tmp);

  loadData::loadPtreeValue(pt, maxSingleEventIterations_, fieldName + ".maxSingleEventIterations", verbose);
  loadData::loadPtreeValue(pt, useTrajectorySpreadingController_, fieldName + ".useTrajectorySpreadingController", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
  }
}

}  // namespace ocs2
