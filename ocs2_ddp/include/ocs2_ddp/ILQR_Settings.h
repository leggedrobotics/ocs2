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

#include <ocs2_ddp/DDP_Settings.h>

namespace ocs2 {

/**
 * This structure contains the settings for the ILQR algorithm.
 */
class ILQR_Settings {
 public:
  /**
   * Default constructor.
   */
  ILQR_Settings()
      : ddpSettings_()

  {}

  /**
   * This function loads the "ILQR_Settings" variables from a config file. This file contains the settings for the SQL and OCS2 algorithms.
   * Here, we use the INFO format which was created specifically for the property tree library (refer to www.goo.gl/fV3yWA).
   *
   * It has the following format:	<br>
   * ILQR	<br>
   * {	<br>
   *   maxIterationILQR        value		<br>
   *   minLearningRateILQR     value		<br>
   *   maxLearningRateILQR     value		<br>
   *   minRelCostILQR          value		<br>
   *   (and so on for the other fields)	<br>
   * }	<br>
   *
   * If a value for a specific field is not defined it will set to the default value defined in "ILQR_Settings".
   *
   * @param [in] filename: File name which contains the configuration data.
   * @param [in] fieldName: Field name which contains the configuration data (the default is ilqr).
   * @param [in] verbose: Flag to determine whether to print out the loaded settings or not (The default is true).
   */
  void loadSettings(const std::string& filename, const std::string& fieldName = "ilqr", bool verbose = true);

 public:
  /****************
   *** Variables **
   ****************/

  /** This structure contains the settings for DDP algorithms. */
  DDP_Settings ddpSettings_;

};  // end of ILQR_Settings class

inline void ILQR_Settings::loadSettings(const std::string& filename, const std::string& fieldName /*= ilqr*/, bool verbose /*= true*/) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  ddpSettings_.loadSettings(filename, fieldName + ".ddp", verbose);

  if (verbose) {
    std::cerr << std::endl << " #### ILQR Settings: " << std::endl;
    std::cerr << " #### =============================================================================" << std::endl;
  }

  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
  }
}

}  // namespace ocs2
