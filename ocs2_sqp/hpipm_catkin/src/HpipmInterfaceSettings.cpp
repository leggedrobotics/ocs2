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

#include "hpipm_catkin/HpipmInterfaceSettings.h"

#include <ocs2_core/misc/LoadData.h>

namespace ocs2 {
namespace hpipm_interface {

std::ostream& operator<<(std::ostream& stream, const Settings& settings) {
  Settings defaultSettings;

  stream << "\n #### HPIPM Settings:";
  stream << "\n #### =============================================================================\n";
  loadData::printValue(stream, settings.hpipmMode, "mode", settings.hpipmMode != defaultSettings.hpipmMode);
  loadData::printValue(stream, settings.iter_max, "iter_max", settings.iter_max != defaultSettings.iter_max);
  loadData::printValue(stream, settings.alpha_min, "alpha_min", settings.alpha_min != defaultSettings.alpha_min);
  loadData::printValue(stream, settings.mu0, "mu0", settings.mu0 != defaultSettings.mu0);
  loadData::printValue(stream, settings.tol_stat, "tol_stat", settings.tol_stat != defaultSettings.tol_stat);
  loadData::printValue(stream, settings.tol_eq, "tol_eq", settings.tol_eq != defaultSettings.tol_eq);
  loadData::printValue(stream, settings.tol_ineq, "tol_ineq", settings.tol_ineq != defaultSettings.tol_ineq);
  loadData::printValue(stream, settings.tol_comp, "tol_comp", settings.tol_comp != defaultSettings.tol_comp);
  loadData::printValue(stream, settings.reg_prim, "reg_prim", settings.reg_prim != defaultSettings.reg_prim);
  loadData::printValue(stream, settings.warm_start, "warm_start", settings.warm_start != defaultSettings.warm_start);
  loadData::printValue(stream, settings.pred_corr, "pred_corr", settings.pred_corr != defaultSettings.pred_corr);
  loadData::printValue(stream, settings.ric_alg, "ric_alg", settings.ric_alg != defaultSettings.ric_alg);
  stream << " #### =============================================================================" << std::endl;
  return stream;
}

}  // namespace hpipm_interface
}  // namespace ocs2