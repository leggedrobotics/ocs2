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

#include "ocs2_slp/pipg/PipgSettings.h"

#include <iostream>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

namespace ocs2 {
namespace pipg {

Settings loadSettings(const std::string& filename, const std::string& fieldName, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  Settings settings;

  if (verbose) {
    std::cerr << " #### PIPG Settings: {\n";
  }

  loadData::loadPtreeValue(pt, settings.maxNumIterations, fieldName + ".maxNumIterations", verbose);
  loadData::loadPtreeValue(pt, settings.absoluteTolerance, fieldName + ".absoluteTolerance", verbose);
  loadData::loadPtreeValue(pt, settings.relativeTolerance, fieldName + ".relativeTolerance", verbose);

  loadData::loadPtreeValue(pt, settings.lowerBoundH, fieldName + ".lowerBoundH", verbose);

  loadData::loadPtreeValue(pt, settings.checkTerminationInterval, fieldName + ".checkTerminationInterval", verbose);
  loadData::loadPtreeValue(pt, settings.displayShortSummary, fieldName + ".displayShortSummary", verbose);

  if (verbose) {
    std::cerr << " #### }\n";
  }

  return settings;
}

}  // namespace pipg
}  // namespace ocs2