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

#include <ocs2_oc/oc_solver/ModeScheduleManager.h>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelLogicRulesBase.h"

namespace switched_model {

/**
 * Manages the ModeSchedule for switched model.
 */
class SwitchedModelModeScheduleManager : public ocs2::ModeScheduleManager<STATE_DIM, INPUT_DIM> {
 public:
  using Base = ocs2::ModeScheduleManager<STATE_DIM, INPUT_DIM>;

  explicit SwitchedModelModeScheduleManager(ocs2::ModeSchedule modeSchedule);

  ~SwitchedModelModeScheduleManager() override = default;

  contact_flag_t getContactFlags(scalar_t time) const;

  std::shared_ptr<SwitchedModelLogicRulesBase> getLogicRules() { return logicRulesPtr_; };

 private:
  void preSolverRunImpl(scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                        const ocs2::CostDesiredTrajectories& costDesiredTrajectory, ocs2::ModeSchedule& modeSchedule) override;

 private:
  std::shared_ptr<SwitchedModelLogicRulesBase> logicRulesPtr_;
};

}  // namespace switched_model
