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

#include "ocs2_ddp/search_strategy/StrategySettings.h"

#include <algorithm>
#include <unordered_map>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
namespace search_strategy {

std::string toString(Type strategy) {
  static const std::unordered_map<Type, std::string> strategyMap{{Type::LINE_SEARCH, "LINE_SEARCH"},
                                                                 {Type::LEVENBERG_MARQUARDT, "LEVENBERG_MARQUARDT"}};
  return strategyMap.at(strategy);
}

Type fromString(std::string name) {
  static const std::unordered_map<std::string, Type> strategyMap{{"LINE_SEARCH", Type::LINE_SEARCH},
                                                                 {"LEVENBERG_MARQUARDT", Type::LEVENBERG_MARQUARDT}};
  std::transform(name.begin(), name.end(), name.begin(), ::toupper);
  return strategyMap.at(name);
}

}  // namespace search_strategy

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
namespace line_search {

Settings load(const std::string& filename, const std::string& fieldName, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);
  if (verbose) {
    std::cerr << " #### LINE_SEARCH Settings: {\n";
  }

  Settings settings;

  loadData::loadPtreeValue(pt, settings.minStepLength_, fieldName + ".minStepLength", verbose);
  loadData::loadPtreeValue(pt, settings.maxStepLength_, fieldName + ".maxStepLength", verbose);
  loadData::loadPtreeValue(pt, settings.contractionRate_, fieldName + ".contractionRate", verbose);
  loadData::loadPtreeValue(pt, settings.armijoCoefficient_, fieldName + ".armijoCoefficient", verbose);

  std::string hessianCorrectionStrategyName = hessian_correction::toString(settings.hessianCorrectionStrategy_);
  loadData::loadPtreeValue(pt, hessianCorrectionStrategyName, fieldName + ".hessianCorrectionStrategy", verbose);
  settings.hessianCorrectionStrategy_ = hessian_correction::fromString(hessianCorrectionStrategyName);

  loadData::loadPtreeValue(pt, settings.hessianCorrectionMultiple_, fieldName + ".hessianCorrectionMultiple", verbose);

  if (verbose) {
    std::cerr << " #### }" << std::endl;
  }

  return settings;
}

}  // namespace line_search

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
namespace levenberg_marquardt {

Settings load(const std::string& filename, const std::string& fieldName, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);
  if (verbose) {
    std::cerr << " #### LEVENBERG_MARQUARDT Settings: {\n";
  }

  Settings settings;

  loadData::loadPtreeValue(pt, settings.minAcceptedPho_, fieldName + ".minAcceptedPho", verbose);
  loadData::loadPtreeValue(pt, settings.riccatiMultipleDefaultRatio_, fieldName + ".riccatiMultipleDefaultRatio", verbose);
  loadData::loadPtreeValue(pt, settings.riccatiMultipleDefaultFactor_, fieldName + ".riccatiMultipleDefaultFactor", verbose);
  loadData::loadPtreeValue(pt, settings.maxNumSuccessiveRejections_, fieldName + ".maxNumSuccessiveRejections", verbose);
  if (verbose) {
    std::cerr << " #### }" << std::endl;
  }

  return settings;
}

}  // namespace levenberg_marquardt

}  // namespace ocs2
