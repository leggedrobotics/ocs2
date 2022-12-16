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

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/integration/SensitivityIntegrator.h>

#include <hpipm_catkin/HpipmInterfaceSettings.h>

namespace ocs2 {
namespace ipm {

struct Settings {
  // Ipm settings
  size_t ipmIteration = 10;  // Maximum number of IPM iterations
  scalar_t deltaTol = 1e-6;  // Termination condition : RMS update of x(t) and u(t) are both below this value
  scalar_t costTol = 1e-4;   // Termination condition : (cost{i+1} - (cost{i}) < costTol AND constraints{i+1} < g_min

  // Linesearch - step size rules
  scalar_t alpha_decay = 0.5;  // multiply the step size by this factor every time a linesearch step is rejected.
  scalar_t alpha_min = 1e-4;   // terminate linesearch if the attempted step size is below this threshold

  // Linesearch - step acceptance criteria with c = costs, g = the norm of constraint violation, and w = [x; u]
  scalar_t g_max = 1e6;          // (1): IF g{i+1} > g_max REQUIRE g{i+1} < (1-gamma_c) * g{i}
  scalar_t g_min = 1e-6;         // (2): ELSE IF (g{i} < g_min AND g{i+1} < g_min AND dc/dw'{i} * delta_w < 0) REQUIRE armijo condition
  scalar_t armijoFactor = 1e-4;  // Armijo condition: c{i+1} < c{i} + armijoFactor * dc/dw'{i} * delta_w
  scalar_t gamma_c = 1e-6;       // (3): ELSE REQUIRE c{i+1} < (c{i} - gamma_c * g{i}) OR g{i+1} < (1-gamma_c) * g{i}

  // controller type
  bool useFeedbackPolicy = true;            // true to use feedback, false to use feedforward
  bool createValueFunction = false;         // true to store the value function, false to ignore it
  bool computeLagrangeMultipliers = false;  // If set to true to compute the Lagrange multipliers. If set to false the dualFeasibilitiesSSE
                                            // in the PerformanceIndex log is incorrect but it will not affect algorithm correctness.

  // QP subproblem solver settings
  hpipm_interface::Settings hpipmSettings = hpipm_interface::Settings();

  // Discretization method
  scalar_t dt = 0.01;  // user-defined time discretization
  SensitivityIntegratorType integratorType = SensitivityIntegratorType::RK2;

  // Barrier strategy of the primal-dual interior point method. Conventions follows Ipopt.
  scalar_t initialBarrierParameter = 1.0e-02;  // Initial value of the barrier parameter
  scalar_t targetBarrierParameter = 1.0e-04;   // Targer value of the barrier parameter. The barreir will decrease until reaches this value.
  scalar_t barrierReductionCostTol = 1.0e-02;  // Barrier reduction condition : (cost{i+1} - (cost{i}) < costTol
  scalar_t barrierReductionConstraintTol = 1.0e-02;  // Barrier reduction condition : Constraint violations below this value
  scalar_t barrierLinearDecreaseFactor = 0.2;        // Linear decrease factor of the barrier parameter, i.e., mu <- mu * factor.
  scalar_t barrierSuperlinearDecreasePower = 1.5;    // Superlinear decrease factor of the barrier parameter, i.e., mu <- mu ^ factor

  // Initialization of the interior point method. Follows the initialization method of IPOPT
  // (https://coin-or.github.io/Ipopt/OPTIONS.html#OPT_Initialization).
  scalar_t initialSlackLowerBound =
      1.0e-04;  // Minimum value of the initial slack variable. Corresponds to `slack_bound_push` option of IPOPT.
  scalar_t initialDualLowerBound = 1.0e-04;  // Minimum value of the initial dual variable.
  scalar_t initialSlackMarginRate = 0.01;  // Margin rate of the initial slack variables. Corresponds to `slack_bound_frac` option of IPOPT.
  scalar_t initialDualMarginRate = 0.01;   // Margin rate of the initial dual variables.

  // Linesearch for the interior point method.
  scalar_t fractionToBoundaryMargin =
      0.995;  // Margin of the fraction-to-boundary-rule for the step size selection. Correcponds to `tau_min` option of IPOPT.
  bool usePrimalStepSizeForDual = true;  // If true, the primal step size is also used as the dual step size.

  // Printing
  bool printSolverStatus = false;      // Print HPIPM status after solving the QP subproblem
  bool printSolverStatistics = false;  // Print benchmarking of the multiple shooting method
  bool printLinesearch = false;        // Print linesearch information

  // Threading
  size_t nThreads = 4;
  int threadPriority = 50;
};

/**
 * Loads the multiple shooting IPM settings from a given file.
 *
 * @param [in] filename: File name which contains the configuration data.
 * @param [in] fieldName: Field name which contains the configuration data.
 * @param [in] verbose: Flag to determine whether to print out the loaded settings or not.
 * @return The settings
 */
Settings loadSettings(const std::string& filename, const std::string& fieldName = "ipm", bool verbose = true);

}  // namespace ipm
}  // namespace ocs2