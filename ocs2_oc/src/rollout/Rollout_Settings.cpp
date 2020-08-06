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

#include "ocs2_oc/rollout/Rollout_Settings.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

namespace ocs2 {
namespace rollout {

Settings loadSettings(const std::string& filename, const std::string& fieldName, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  Settings settings;

  if (verbose) {
    std::cerr << "\n #### Rollout Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::loadPtreeValue(pt, settings.absTolODE_, fieldName + ".AbsTolODE", verbose);
  loadData::loadPtreeValue(pt, settings.relTolODE_, fieldName + ".RelTolODE", verbose);
  loadData::loadPtreeValue(pt, settings.maxNumStepsPerSecond_, fieldName + ".maxNumStepsPerSecond", verbose);
  loadData::loadPtreeValue(pt, settings.minTimeStep_, fieldName + ".minTimeStep", verbose);

  auto integratorName = integrator_type::toString(settings.integratorType_);  // keep default
  loadData::loadPtreeValue(pt, integratorName, fieldName + ".integratorType", verbose);
  settings.integratorType_ = integrator_type::fromString(integratorName);

  loadData::loadPtreeValue(pt, settings.checkNumericalStability_, fieldName + ".checkNumericalStability", verbose);
  loadData::loadPtreeValue(pt, settings.reconstructInputTrajectory_, fieldName + ".reconstructInputTrajectory", verbose);

  auto rootFindingAlgorithmName = static_cast<int>(settings.rootFindingAlgorithm_);  // keep default
  loadData::loadPtreeValue(pt, rootFindingAlgorithmName, fieldName + ".rootFindingAlgorithm", verbose);
  settings.rootFindingAlgorithm_ = static_cast<RootFinderType>(rootFindingAlgorithmName);

  loadData::loadPtreeValue(pt, settings.maxSingleEventIterations_, fieldName + ".maxSingleEventIterations", verbose);
  loadData::loadPtreeValue(pt, settings.useTrajectorySpreadingController_, fieldName + ".useTrajectorySpreadingController", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
  }

  return settings;
}

}  // namespace rollout
}  // namespace ocs2
