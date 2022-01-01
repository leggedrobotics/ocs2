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

#include "ocs2_ddp/DDP_Settings.h"

#include <algorithm>
#include <iostream>
#include <unordered_map>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

namespace ocs2 {
namespace ddp {

std::string toAlgorithmName(Algorithm type) {
  static const std::unordered_map<Algorithm, std::string> strategyMap{{Algorithm::SLQ, "SLQ"}, {Algorithm::ILQR, "ILQR"}};
  return strategyMap.at(type);
}

Algorithm fromAlgorithmName(std::string name) {
  static const std::unordered_map<std::string, Algorithm> strategyMap{{"SLQ", Algorithm::SLQ}, {"ILQR", Algorithm::ILQR}};
  std::transform(name.begin(), name.end(), name.begin(), ::toupper);
  return strategyMap.at(name);
}

Settings loadSettings(const std::string& filename, const std::string& fieldName, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  Settings settings;

  if (verbose) {
    std::cerr << "\n #### DDP Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }

  std::string algorithmName = toAlgorithmName(settings.algorithm_);
  loadData::loadPtreeValue(pt, algorithmName, fieldName + ".algorithm", verbose);
  settings.algorithm_ = fromAlgorithmName(algorithmName);

  loadData::loadPtreeValue(pt, settings.nThreads_, fieldName + ".nThreads", verbose);
  loadData::loadPtreeValue(pt, settings.threadPriority_, fieldName + ".threadPriority", verbose);

  loadData::loadPtreeValue(pt, settings.maxNumIterations_, fieldName + ".maxNumIterations", verbose);
  loadData::loadPtreeValue(pt, settings.minRelCost_, fieldName + ".minRelCost", verbose);
  loadData::loadPtreeValue(pt, settings.constraintTolerance_, fieldName + ".constraintTolerance", verbose);

  loadData::loadPtreeValue(pt, settings.displayInfo_, fieldName + ".displayInfo", verbose);
  loadData::loadPtreeValue(pt, settings.displayShortSummary_, fieldName + ".displayShortSummary", verbose);
  loadData::loadPtreeValue(pt, settings.checkNumericalStability_, fieldName + ".checkNumericalStability", verbose);
  loadData::loadPtreeValue(pt, settings.debugPrintRollout_, fieldName + ".debugPrintRollout", verbose);
  loadData::loadPtreeValue(pt, settings.debugCaching_, fieldName + ".debugCaching", verbose);

  loadData::loadPtreeValue(pt, settings.absTolODE_, fieldName + ".AbsTolODE", verbose);
  loadData::loadPtreeValue(pt, settings.relTolODE_, fieldName + ".RelTolODE", verbose);
  loadData::loadPtreeValue(pt, settings.maxNumStepsPerSecond_, fieldName + ".maxNumStepsPerSecond", verbose);
  loadData::loadPtreeValue(pt, settings.timeStep_, fieldName + ".timeStep", verbose);
  auto integratorName = integrator_type::toString(settings.backwardPassIntegratorType_);  // keep default
  loadData::loadPtreeValue(pt, integratorName, fieldName + ".backwardPassIntegratorType", verbose);
  settings.backwardPassIntegratorType_ = integrator_type::fromString(integratorName);

  loadData::loadPtreeValue(pt, settings.constraintPenaltyInitialValue_, fieldName + ".constraintPenaltyInitialValue", verbose);
  loadData::loadPtreeValue(pt, settings.constraintPenaltyIncreaseRate_, fieldName + ".constraintPenaltyIncreaseRate", verbose);

  loadData::loadPtreeValue(pt, settings.preComputeRiccatiTerms_, fieldName + ".preComputeRiccatiTerms", verbose);

  loadData::loadPtreeValue(pt, settings.useFeedbackPolicy_, fieldName + ".useFeedbackPolicy", verbose);

  loadData::loadPtreeValue(pt, settings.riskSensitiveCoeff_, fieldName + ".riskSensitiveCoeff", verbose);

  std::string strategyName = search_strategy::toString(settings.strategy_);
  loadData::loadPtreeValue(pt, strategyName, fieldName + ".strategy", verbose);
  settings.strategy_ = search_strategy::fromString(strategyName);

  switch (settings.strategy_) {
    case search_strategy::Type::LINE_SEARCH: {
      settings.lineSearch_ = line_search::load(filename, fieldName + ".lineSearch", verbose);
      break;
    }
    case search_strategy::Type::LEVENBERG_MARQUARDT: {
      settings.levenbergMarquardt_ = levenberg_marquardt::load(filename, fieldName + ".levenbergMarquardt", verbose);
      break;
    }
  }

  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
  }

  return settings;
}

}  // namespace ddp
}  // namespace ocs2
