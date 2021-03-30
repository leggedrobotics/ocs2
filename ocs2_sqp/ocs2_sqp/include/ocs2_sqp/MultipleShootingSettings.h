//
// Created by rgrandia on 30.03.21.
//

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/integration/SensitivityIntegrator.h>

#include <hpipm_catkin/HpipmInterfaceSettings.h>

namespace ocs2 {
namespace multiple_shooting {

struct Settings {
  // System settings
  size_t n_state = 0;
  size_t n_input = 0;

  // Sqp settings
  size_t sqpIteration = 1;
  scalar_t deltaTol = 1e-6;
  scalar_t alpha_decay = 0.5;
  scalar_t alpha_min = 1e-4;
  scalar_t gamma_c = 1e-6;
  scalar_t g_max = 1e6;
  scalar_t g_min = 1e-6;
  scalar_t costTol = 1e-4;

  // QP subproblem solver settings
  hpipm_interface::Settings hpipmSettings = hpipm_interface::Settings();

  // Discretization method
  scalar_t dt = 0.01;  // user-defined time discretization
  SensitivityIntegratorType integratorType = SensitivityIntegratorType::RK2;

  // Inequality penalty method
  scalar_t inequalityConstraintMu = 0.0;
  scalar_t inequalityConstraintDelta = 1e-6;
  bool projectStateInputEqualityConstraints = true;  // Use a projection method to resolve the state-input constraint Cx+Du+e

  // Printing
  bool printSolverStatus = false;      // Print HPIPM status after solving the QP subproblem
  bool printSolverStatistics = false;  // Print benchmarking of the multiple shooting method
  bool printLinesearch = false;        // Print linesearch information

  // Threading
  size_t nThreads = 4;
  int threadPriority = 50;
};

/**
 * Loads the multiple shooting settings from a given file.
 *
 * @param [in] filename: File name which contains the configuration data.
 * @param [in] fieldName: Field name which contains the configuration data.
 * @param [in] verbose: Flag to determine whether to print out the loaded settings or not.
 * @return The settings
 */
Settings loadSettings(const std::string& filename, const std::string& fieldName = "multiple_shooting", bool verbose = true);

}  // namespace multiple_shooting
}  // namespace ocs2