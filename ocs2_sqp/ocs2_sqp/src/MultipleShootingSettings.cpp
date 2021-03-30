//
// Created by rgrandia on 30.03.21.
//

#include "ocs2_sqp/MultipleShootingSettings.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

namespace ocs2 {
namespace multiple_shooting {
Settings loadSettings(const std::string& filename, const std::string& fieldName, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  Settings settings;

  if (verbose) {
    std::cerr << "\n #### Multiple Shooting Settings:";
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::loadPtreeValue(pt, settings.n_state, fieldName + ".n_state", verbose);
  loadData::loadPtreeValue(pt, settings.n_input, fieldName + ".n_input", verbose);
  loadData::loadPtreeValue(pt, settings.sqpIteration, fieldName + ".sqpIteration", verbose);
  loadData::loadPtreeValue(pt, settings.deltaTol, fieldName + ".deltaTol", verbose);
  loadData::loadPtreeValue(pt, settings.alpha_decay, fieldName + ".alpha_decay", verbose);
  loadData::loadPtreeValue(pt, settings.alpha_min, fieldName + ".alpha_min", verbose);
  loadData::loadPtreeValue(pt, settings.gamma_c, fieldName + ".gamma_c", verbose);
  loadData::loadPtreeValue(pt, settings.g_max, fieldName + ".g_max", verbose);
  loadData::loadPtreeValue(pt, settings.g_min, fieldName + ".g_min", verbose);
  loadData::loadPtreeValue(pt, settings.costTol, fieldName + ".costTol", verbose);
  loadData::loadPtreeValue(pt, settings.dt, fieldName + ".dt", verbose);
  auto integratorName = sensitivity_integrator::toString(settings.integratorType);
  loadData::loadPtreeValue(pt, integratorName, fieldName + ".integratorType", verbose);
  settings.integratorType = sensitivity_integrator::fromString(integratorName);
  loadData::loadPtreeValue(pt, settings.inequalityConstraintMu, fieldName + ".inequalityConstraintMu", verbose);
  loadData::loadPtreeValue(pt, settings.inequalityConstraintDelta, fieldName + ".inequalityConstraintDelta", verbose);
  loadData::loadPtreeValue(pt, settings.projectStateInputEqualityConstraints, fieldName + ".projectStateInputEqualityConstraints", verbose);
  loadData::loadPtreeValue(pt, settings.printSolverStatus, fieldName + ".printSolverStatus", verbose);
  loadData::loadPtreeValue(pt, settings.printSolverStatistics, fieldName + ".printSolverStatistics", verbose);
  loadData::loadPtreeValue(pt, settings.printLinesearch, fieldName + ".printLinesearch", verbose);
  loadData::loadPtreeValue(pt, settings.nThreads, fieldName + ".nThreads", verbose);
  loadData::loadPtreeValue(pt, settings.threadPriority, fieldName + ".threadPriority", verbose);

  if (verbose) {
    std::cerr << settings.hpipmSettings;
    std::cerr << " #### =============================================================================" << std::endl;
  }

  return settings;
}
}  // namespace multiple_shooting
}  // namespace ocs2