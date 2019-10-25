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

#include <ocs2_core/Dimensions.h>

#include <ocs2_ddp_base/DDP_Settings.h>

namespace ocs2 {

/**
 * This structure contains the settings for the SLQ algorithm.
 */
class SLQ_Settings {
 public:
  using RICCATI_INTEGRATOR_TYPE = Dimensions<0, 0>::RiccatiIntegratorType;

  /**
   * Default constructor.
   */
  SLQ_Settings()
      : useNominalTimeForBackwardPass_(false),
        preComputeRiccatiTerms_(true),
        RiccatiIntegratorType_(RICCATI_INTEGRATOR_TYPE::ODE45),
        adams_integrator_dt_(0.001),
        ddpSettings_()

  {}

  /**
   * This function loads the "SLQ_Settings" variables from a config file. This file contains the settings for the SQL and OCS2 algorithms.
   * Here, we use the INFO format which was created specifically for the property tree library (refer to www.goo.gl/fV3yWA).
   *
   * It has the following format:	<br>
   * slq	<br>
   * {	<br>
   *   maxIterationSLQ        value		<br>
   *   minLearningRateSLQ     value		<br>
   *   maxLearningRateSLQ     value		<br>
   *   minRelCostSLQ          value		<br>
   *   (and so on for the other fields)	<br>
   * }	<br>
   *
   * If a value for a specific field is not defined it will set to the default value defined in "SLQ_Settings".
   *
   * @param [in] filename: File name which contains the configuration data.
   * @param [in] fieldName: Field name which contains the configuration data (the default is slq).
   * @param [in] verbose: Flag to determine whether to print out the loaded settings or not (the default is true).
   */
  void loadSettings(const std::string& filename, const std::string& fieldName = "slq", bool verbose = true);

  /**
   * Helper function for loading setting fields
   *
   * @param [in] pt: Property_tree.
   * @param [in] prefix: Field name prefix.
   * @param [in] fieldName: Field name.
   * @param [in] verbose: Print loaded settings if true.
   */
  template <typename T>
  void loadField(const boost::property_tree::ptree& pt, const std::string& prefix, const std::string& fieldName, T& field, bool verbose);

 public:
  /****************
   *** Variables **
   ****************/

  /** If true, SLQ solves the backward path over the nominal time trajectory. */
  bool useNominalTimeForBackwardPass_;
  /** If true, terms of the Riccati equation will be precomputed before interpolation in the flowmap */
  bool preComputeRiccatiTerms_;
  /** Riccati integrator type. */
  size_t RiccatiIntegratorType_;
  /** Adams integrator's time step. */
  double adams_integrator_dt_;

  /** This structure contains the settings for DDP algorithms. */
  DDP_Settings ddpSettings_;

};  // end of SLQ_Settings class

template <typename T>
inline void SLQ_Settings::loadField(const boost::property_tree::ptree& pt, const std::string& prefix, const std::string& fieldName,
                                    T& field, bool verbose) {
  std::string comment;
  try {
    field = pt.get<T>(prefix + "." + fieldName);
  } catch (const std::exception& e) {
    comment = "   \t(default)";
  }

  if (verbose) {
    int fill = std::max<int>(0, 36 - static_cast<int>(fieldName.length()));
    std::cerr << " #### Option loader : option '" << fieldName << "' " << std::string(fill, '.') << " " << field << comment << std::endl;
  }
}

inline void SLQ_Settings::loadSettings(const std::string& filename, const std::string& fieldName /*= slq*/, bool verbose /*= true*/) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  ddpSettings_.loadSettings(filename, fieldName + ".ddp", verbose);

  if (verbose) {
    std::cerr << std::endl << " #### SLQ Settings: " << std::endl;
    std::cerr << " #### =============================================================================" << std::endl;
  }

  loadField(pt, fieldName, "useNominalTimeForBackwardPass", useNominalTimeForBackwardPass_, verbose);
  loadField(pt, fieldName, "preComputeRiccatiTerms", preComputeRiccatiTerms_, verbose);
  loadField(pt, fieldName, "RiccatiIntegratorType", RiccatiIntegratorType_, verbose);
  loadField(pt, fieldName, "adams_integrator_dt", adams_integrator_dt_, verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
  }
}

}  // namespace ocs2
