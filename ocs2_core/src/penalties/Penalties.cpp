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

#include "ocs2_core/penalties/Penalties.h"

#include <ocs2_core/misc/LoadData.h>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace ocs2 {
namespace loadData {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <>
void loadPenaltyConfig<augmented::SmoothAbsolutePenalty::Config>(const std::string& fileName, const std::string& fieldName,
                                                                 augmented::SmoothAbsolutePenalty::Config& config, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(fileName, pt);

  if (verbose) {
    std::cerr << "\n #### " << fieldName;
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::loadPtreeValue(pt, config.scale, fieldName + ".scale", verbose);
  loadData::loadPtreeValue(pt, config.relaxation, fieldName + ".relaxation", verbose);
  loadData::loadPtreeValue(pt, config.stepSize, fieldName + ".stepSize", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================\n" << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <>
void loadPenaltyConfig<augmented::QuadraticPenalty::Config>(const std::string& fileName, const std::string& fieldName,
                                                            augmented::QuadraticPenalty::Config& config, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(fileName, pt);

  if (verbose) {
    std::cerr << "\n #### " << fieldName;
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::loadPtreeValue(pt, config.scale, fieldName + ".scale", verbose);
  loadData::loadPtreeValue(pt, config.stepSize, fieldName + ".stepSize", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================\n" << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <>
void loadPenaltyConfig<augmented::ModifiedRelaxedBarrierPenalty::Config>(const std::string& fileName, const std::string& fieldName,
                                                                         augmented::ModifiedRelaxedBarrierPenalty::Config& config,
                                                                         bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(fileName, pt);

  if (verbose) {
    std::cerr << "\n #### " << fieldName;
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::loadPtreeValue(pt, config.scale, fieldName + ".scale", verbose);
  loadData::loadPtreeValue(pt, config.relaxation, fieldName + ".relaxation", verbose);
  loadData::loadPtreeValue(pt, config.stepSize, fieldName + ".stepSize", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================\n" << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <>
void loadPenaltyConfig<augmented::SlacknessSquaredHingePenalty::Config>(const std::string& fileName, const std::string& fieldName,
                                                                        augmented::SlacknessSquaredHingePenalty::Config& config,
                                                                        bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(fileName, pt);

  if (verbose) {
    std::cerr << "\n #### " << fieldName;
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::loadPtreeValue(pt, config.scale, fieldName + ".scale", verbose);
  loadData::loadPtreeValue(pt, config.stepSize, fieldName + ".stepSize", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================\n" << std::endl;
  }
}

}  // namespace loadData
}  // namespace ocs2
