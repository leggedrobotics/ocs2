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

#include "ocs2_ipm/IpmSettings.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

#include <stdexcept>

namespace ocs2 {
namespace ipm {

Settings loadSettings(const std::string& filename, const std::string& fieldName, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  Settings settings;

  if (verbose) {
    std::cerr << "\n #### Multiple-Shooting IPM Settings:";
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::loadPtreeValue(pt, settings.ipmIteration, fieldName + ".ipmIteration", verbose);
  loadData::loadPtreeValue(pt, settings.deltaTol, fieldName + ".deltaTol", verbose);
  loadData::loadPtreeValue(pt, settings.alpha_decay, fieldName + ".alpha_decay", verbose);
  loadData::loadPtreeValue(pt, settings.alpha_min, fieldName + ".alpha_min", verbose);
  loadData::loadPtreeValue(pt, settings.gamma_c, fieldName + ".gamma_c", verbose);
  loadData::loadPtreeValue(pt, settings.g_max, fieldName + ".g_max", verbose);
  loadData::loadPtreeValue(pt, settings.g_min, fieldName + ".g_min", verbose);
  loadData::loadPtreeValue(pt, settings.armijoFactor, fieldName + ".armijoFactor", verbose);
  loadData::loadPtreeValue(pt, settings.costTol, fieldName + ".costTol", verbose);
  loadData::loadPtreeValue(pt, settings.dt, fieldName + ".dt", verbose);
  loadData::loadPtreeValue(pt, settings.useFeedbackPolicy, fieldName + ".useFeedbackPolicy", verbose);
  loadData::loadPtreeValue(pt, settings.createValueFunction, fieldName + ".createValueFunction", verbose);
  loadData::loadPtreeValue(pt, settings.computeLagrangeMultipliers, fieldName + ".computeLagrangeMultipliers", verbose);
  auto integratorName = sensitivity_integrator::toString(settings.integratorType);
  loadData::loadPtreeValue(pt, integratorName, fieldName + ".integratorType", verbose);
  settings.integratorType = sensitivity_integrator::fromString(integratorName);
  loadData::loadPtreeValue(pt, settings.initialBarrierParameter, fieldName + ".initialBarrierParameter", verbose);
  loadData::loadPtreeValue(pt, settings.targetBarrierParameter, fieldName + ".targetBarrierParameter", verbose);
  loadData::loadPtreeValue(pt, settings.barrierReductionCostTol, fieldName + ".barrierReductionCostTol ", verbose);
  loadData::loadPtreeValue(pt, settings.barrierReductionConstraintTol, fieldName + ".barrierReductionConstraintTol", verbose);
  loadData::loadPtreeValue(pt, settings.barrierLinearDecreaseFactor, fieldName + ".barrierLinearDecreaseFactor", verbose);
  loadData::loadPtreeValue(pt, settings.barrierSuperlinearDecreasePower, fieldName + ".barrierSuperlinearDecreasePower", verbose);
  loadData::loadPtreeValue(pt, settings.fractionToBoundaryMargin, fieldName + ".fractionToBoundaryMargin", verbose);
  loadData::loadPtreeValue(pt, settings.usePrimalStepSizeForDual, fieldName + ".usePrimalStepSizeForDual", verbose);
  loadData::loadPtreeValue(pt, settings.initialSlackLowerBound, fieldName + ".initialSlackLowerBound", verbose);
  loadData::loadPtreeValue(pt, settings.initialDualLowerBound, fieldName + ".initialDualLowerBound", verbose);
  loadData::loadPtreeValue(pt, settings.initialSlackMarginRate, fieldName + ".initialSlackMarginRate", verbose);
  loadData::loadPtreeValue(pt, settings.initialDualMarginRate, fieldName + ".initialDualMarginRate", verbose);
  loadData::loadPtreeValue(pt, settings.printSolverStatus, fieldName + ".printSolverStatus", verbose);
  loadData::loadPtreeValue(pt, settings.printSolverStatistics, fieldName + ".printSolverStatistics", verbose);
  loadData::loadPtreeValue(pt, settings.printLinesearch, fieldName + ".printLinesearch", verbose);
  loadData::loadPtreeValue(pt, settings.nThreads, fieldName + ".nThreads", verbose);
  loadData::loadPtreeValue(pt, settings.threadPriority, fieldName + ".threadPriority", verbose);

  if (settings.initialSlackLowerBound <= 0.0) {
    throw std::runtime_error("[MultipleShootingIpmSettings] initialSlackLowerBound must be positive!");
  }
  if (settings.initialDualLowerBound <= 0.0) {
    throw std::runtime_error("[MultipleShootingIpmSettings] initialDualLowerBound must be positive!");
  }
  if (settings.initialSlackMarginRate < 0.0) {
    throw std::runtime_error("[MultipleShootingIpmSettings] initialSlackMarginRate must be non-negative!");
  }
  if (settings.initialDualMarginRate < 0.0) {
    throw std::runtime_error("[MultipleShootingIpmSettings] initialDualMarginRate must be non-negative!");
  }
  if (settings.fractionToBoundaryMargin <= 0.0 || settings.fractionToBoundaryMargin > 1.0) {
    throw std::runtime_error("[MultipleShootingIpmSettings] fractionToBoundaryMargin must be positive and no more than 1.0!");
  }

  if (verbose) {
    std::cerr << settings.hpipmSettings;
    std::cerr << " #### =============================================================================" << std::endl;
  }

  return settings;
}
}  // namespace ipm
}  // namespace ocs2