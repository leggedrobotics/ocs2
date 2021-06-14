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

#include <iostream>
#include <string>

#include <ocs2_core/Types.h>
#include <ocs2_core/integration/Integrator.h>
#include "ocs2_oc/rollout/RootFinderType.h"

namespace ocs2 {
namespace rollout {

/**
 * This structure contains the settings for forward rollout algorithms.
 */
struct Settings {
  /** This value determines the absolute tolerance error for ode solvers. */
  scalar_t absTolODE = 1e-9;
  /** This value determines the relative tolerance error for ode solvers. */
  scalar_t relTolODE = 1e-6;
  /** This value determines the maximum number of integration points per a second for ode solvers. */
  size_t maxNumStepsPerSecond = 10000;
  /** The integration time step used in the fixed time-step rollout methods */
  scalar_t timeStep = 1e-2;
  /** Rollout integration scheme type */
  IntegratorType integratorType = IntegratorType::ODE45;

  /** Whether to check that the rollout is numerically stable */
  bool checkNumericalStability = false;
  /** Whether to run controller again after integration to construct input trajectory */
  bool reconstructInputTrajectory = true;

  /** Which of the RootFinding algorithms to use in StateRollout
   * 		0:		Anderson & Björck		(default)
   * 		1:		Pegasus
   * 		2:		Illinois
   * 		3:		Regula Falsi
   */
  RootFinderType rootFindingAlgorithm = RootFinderType::ANDERSON_BJORCK;
  /** This value determines the maximum number of iterations, per event, allowed in state triggered rollout to find
   *  the guard surface zero crossing.  */
  int maxSingleEventIterations = 10;
  /** Whether to use the trajectory spreading controller in state triggered rollout */
  bool useTrajectorySpreadingController = false;
};

/**
 * This function loads the "rollout::Settings" variables from a config file. This file contains the settings for the Rollout algorithms.
 * Here, we use the INFO format which was created specifically for the property tree library (refer to www.goo.gl/fV3yWA).
 *
 * It has the following format: <br>
 * rollout  <br>
 * {  <br>
 *   absTolODE                value   <br>
 *   relTolODE                value   <br>
 *   maxNumStepsPerSecond     value   <br>
 *   timeStep                 value   <br>
 *   (and so on for the other fields) <br>
 * }  <br>
 *
 * If a value for a specific field is not defined it will set to the default value defined in "rollout::Settings".
 *
 * @param [in] filename: File name which contains the configuration data.
 * @param [in] fieldName: Field name which contains the configuration data.
 * @param [in] verbose: Flag to determine whether to print out the loaded settings or not (The default is true).
 */
Settings loadSettings(const std::string& filename, const std::string& fieldName = "rollout", bool verbose = true);

}  // namespace rollout
}  // namespace ocs2
