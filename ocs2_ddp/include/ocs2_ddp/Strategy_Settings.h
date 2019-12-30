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
#include <unordered_map>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/misc/LoadData.h>

namespace ocs2 {

/**
 * @brief The DDP strategy enum
 * Enum used in selecting either line search strategy or trust region strategy.
 */
enum class DDP_Strategy { LINE_SEARCH, TRUST_REGION };

/**
 * Get string name of DDP_Strategy type
 * @param [in] strategy: DDP_Strategy type enum
 */
std::string toString(DDP_Strategy strategy) {
  static const std::unordered_map<DDP_Strategy, std::string> strategyMap{{DDP_Strategy::LINE_SEARCH, "LINE_SEARCH"},
                                                                         {DDP_Strategy::TRUST_REGION, "TRUST_REGION"}};
  return strategyMap.at(strategy);
}

/**
 * Get DDP_Strategy type from string name, useful for reading config file
 * @param [in] name: DDP_Strategy name
 */
DDP_Strategy fromString(std::string name) {
  static const std::unordered_map<std::string, DDP_Strategy> strategyMap{{"LINE_SEARCH", DDP_Strategy::LINE_SEARCH},
                                                                         {"TRUST_REGION", DDP_Strategy::TRUST_REGION}};
  return strategyMap.at(name);
}

/**
 * This structure contains the settings for the line search strategy.
 */
struct Line_Search {
  /** Minimum step length of line-search strategy. */
  double minStepLength_ = 0.05;
  /** Maximum step length of line-search strategy. */
  double maxStepLength_ = 1.0;
  /** Line-search strategy contraction rate. */
  double contractionRate_ = 0.5;
  /** If true DDP makes sure that PSD matrices remain PSD which increases the numerical stability at the expense of extra computation.*/
  bool useMakePSD_ = true;
  /** Add diagonal term to Riccati backward pass for numerical stability. This process is only used when useMakePSD_ set to false.*/
  double addedRiccatiDiagonal_ = 1e-5;

  /**
   * This function loads the "Line_Search" variables from a config file.
   * Here, we use the INFO format which was created specifically for the property tree library (refer to www.goo.gl/fV3yWA).
   * @param [in] filename: File name which contains the configuration data.
   * @param [in] fieldName: Field name which contains the configuration data.
   * @param [in] verbose: Flag to determine whether to print out the loaded settings or not (The default is true).
   */
  void loadSettings(const std::string& filename, const std::string& fieldName, bool verbose = true) {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);
    if (verbose) {
      std::cerr << " #### LINE_SEARCH Settings: {" << std::endl;
    }
    loadData::loadPtreeValue(pt, minStepLength_, fieldName + ".minStepLength", verbose);
    loadData::loadPtreeValue(pt, maxStepLength_, fieldName + ".maxStepLength", verbose);
    loadData::loadPtreeValue(pt, contractionRate_, fieldName + ".contractionRate", verbose);
    loadData::loadPtreeValue(pt, useMakePSD_, fieldName + ".useMakePSD", verbose);
    loadData::loadPtreeValue(pt, addedRiccatiDiagonal_, fieldName + ".addedRiccatiDiagonal", verbose);
    if (verbose) {
      std::cerr << " #### }" << std::endl;
    }
  }
};  // end of Line_Search

/**
 * This structure contains the settings for the trust region strategy.
 */
struct Trust_Region {
  /** M. */

  /**
   * This function loads the "Trust_Region" variables from a config file.
   * Here, we use the INFO format which was created specifically for the property tree library (refer to www.goo.gl/fV3yWA).
   * @param [in] filename: File name which contains the configuration data.
   * @param [in] fieldName: Field name which contains the configuration data.
   * @param [in] verbose: Flag to determine whether to print out the loaded settings or not (The default is true).
   */
  void loadSettings(const std::string& filename, const std::string& fieldName, bool verbose = true) {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);
    if (verbose) {
      std::cerr << " #### TRUST_REGION Settings: {" << std::endl;
    }
    if (verbose) {
      std::cerr << " #### }" << std::endl;
    }
  }
};  // end of Line_Search

}  // namespace ocs2
