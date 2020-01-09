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
#include <ocs2_core/OCS2NumericTraits.h>
#include <ocs2_core/misc/LoadData.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief The DDP strategy enum
 * Enum used in selecting either LINE_SEARCH, LEVENBERG_MARQUARDT, or TRUST_REGION strategies.
 */
enum class DDP_Strategy { LINE_SEARCH, LEVENBERG_MARQUARDT };

namespace ddp_strategy {
/**
 * Get string name of DDP_Strategy type
 * @param [in] strategy: DDP_Strategy type enum
 */
std::string toString(DDP_Strategy strategy) {
  static const std::unordered_map<DDP_Strategy, std::string> strategyMap{{DDP_Strategy::LINE_SEARCH, "LINE_SEARCH"},
                                                                         {DDP_Strategy::LEVENBERG_MARQUARDT, "LEVENBERG_MARQUARDT"}};
  return strategyMap.at(strategy);
}

/**
 * Get DDP_Strategy type from string name, useful for reading config file
 * @param [in] name: DDP_Strategy name
 */
DDP_Strategy fromString(std::string name) {
  static const std::unordered_map<std::string, DDP_Strategy> strategyMap{{"LINE_SEARCH", DDP_Strategy::LINE_SEARCH},
                                                                         {"LEVENBERG_MARQUARDT", DDP_Strategy::LEVENBERG_MARQUARDT}};
  return strategyMap.at(name);
}
}  // namespace ddp_strategy

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief The DDP strategy enum
 * Enum used in selecting either DIAGONAL_SHIFT, CHOLESKY_MODIFICATION, EIGENVALUE_MODIFICATION, or GERSHGORIN_MODIFICATION strategies.
 */
enum class Hessian_Correction { DIAGONAL_SHIFT, CHOLESKY_MODIFICATION, EIGENVALUE_MODIFICATION, GERSHGORIN_MODIFICATION };

namespace hessian_correction {
/**
 * Get string name of Hessian_Correction type
 * @param [in] strategy: Hessian_Correction type enum
 */
std::string toString(Hessian_Correction strategy) {
  static const std::unordered_map<Hessian_Correction, std::string> strategyMap{
      {Hessian_Correction::DIAGONAL_SHIFT, "DIAGONAL_SHIFT"},
      {Hessian_Correction::CHOLESKY_MODIFICATION, "CHOLESKY_MODIFICATION"},
      {Hessian_Correction::EIGENVALUE_MODIFICATION, "EIGENVALUE_MODIFICATION"},
      {Hessian_Correction::GERSHGORIN_MODIFICATION, "GERSHGORIN_MODIFICATION"}};
  return strategyMap.at(strategy);
}

/**
 * Get Hessian_Correction type from string name, useful for reading config file
 * @param [in] name: Hessian_Correction name
 */
Hessian_Correction fromString(std::string name) {
  static const std::unordered_map<std::string, Hessian_Correction> strategyMap{
      {"DIAGONAL_SHIFT", Hessian_Correction::DIAGONAL_SHIFT},
      {"CHOLESKY_MODIFICATION", Hessian_Correction::CHOLESKY_MODIFICATION},
      {"EIGENVALUE_MODIFICATION", Hessian_Correction::EIGENVALUE_MODIFICATION},
      {"GERSHGORIN_MODIFICATION", Hessian_Correction::GERSHGORIN_MODIFICATION}};
  return strategyMap.at(name);
}
}  // namespace hessian_correction

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * This structure contains the settings for the Line-Search strategy.
 */
struct Line_Search {
  /** Minimum step length of line-search strategy. */
  double minStepLength_ = 0.05;
  /** Maximum step length of line-search strategy. */
  double maxStepLength_ = 1.0;
  /** Line-search strategy contraction rate. */
  double contractionRate_ = 0.5;
  /** Armijo coefficient, c defined as f(u + a*p) < f(u) + c*a dfdu.dot(p)  */
  double armijoCoefficient_ = 1e-4;
  /** The Hessian correction strategy. */
  Hessian_Correction hessianCorrectionStrategy_ = Hessian_Correction::DIAGONAL_SHIFT;
  /** The multiple used for correcting the Hessian for numerical stability of the Riccati backward pass.*/
  double hessianCorrectionMultiple_ = OCS2NumericTraits<double>::limitEpsilon();

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
    loadData::loadPtreeValue(pt, armijoCoefficient_, fieldName + ".armijoCoefficient", verbose);

    std::string hessianCorrectionStrategyName = hessian_correction::toString(hessianCorrectionStrategy_);
    loadData::loadPtreeValue(pt, hessianCorrectionStrategyName, fieldName + ".hessianCorrectionStrategy", verbose);
    hessianCorrectionStrategy_ = hessian_correction::fromString(hessianCorrectionStrategyName);

    loadData::loadPtreeValue(pt, hessianCorrectionMultiple_, fieldName + ".hessianCorrectionMultiple", verbose);

    if (verbose) {
      std::cerr << " #### }" << std::endl;
    }
  }
};  // end of Line_Search

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * This structure contains the settings for the Levenberg-Marquardt strategy.
 */
struct Levenberg_Marquardt {
  /** Minimum pho (the ratio between actual reduction and predicted reduction) to accept the iteration's solution.
   * minAcceptedPho_ should be [0, 0.25);
   * */
  double minAcceptedPho_ = 0.25;
  /** The default ratio of geometric progression for Riccati multiple. */
  double riccatiMultipleDefaultRatio_ = 2.0;
  /** The default scalar-factor of geometric progression for Riccati multiple. */
  double riccatiMultipleDefaultFactor_ = 1e-6;
  /** Maximum number of successive rejections of the iteration's solution. */
  size_t maxNumSuccessiveRejections_ = 5;

  /**
   * This function loads the "Levenberg_Marquardt" variables from a config file.
   * Here, we use the INFO format which was created specifically for the property tree library (refer to www.goo.gl/fV3yWA).
   * @param [in] filename: File name which contains the configuration data.
   * @param [in] fieldName: Field name which contains the configuration data.
   * @param [in] verbose: Flag to determine whether to print out the loaded settings or not (The default is true).
   */
  void loadSettings(const std::string& filename, const std::string& fieldName, bool verbose = true) {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);
    if (verbose) {
      std::cerr << " #### LEVENBERG_MARQUARDT Settings: {" << std::endl;
    }
    loadData::loadPtreeValue(pt, minAcceptedPho_, fieldName + ".minAcceptedPho", verbose);
    loadData::loadPtreeValue(pt, riccatiMultipleDefaultRatio_, fieldName + ".riccatiMultipleDefaultRatio", verbose);
    loadData::loadPtreeValue(pt, riccatiMultipleDefaultFactor_, fieldName + ".riccatiMultipleDefaultFactor", verbose);
    loadData::loadPtreeValue(pt, maxNumSuccessiveRejections_, fieldName + ".maxNumSuccessiveRejections", verbose);
    if (verbose) {
      std::cerr << " #### }" << std::endl;
    }
  }
};  // end of Levenberg_Marquardt

}  // namespace ocs2
