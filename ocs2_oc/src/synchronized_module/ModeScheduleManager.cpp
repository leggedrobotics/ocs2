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

#include "ocs2_oc/synchronized_module/ModeScheduleManager.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModeScheduleManager::ModeScheduleManager(ModeSchedule modeSchedule)
    : modeSchedule_(std::move(modeSchedule)), modeScheduleBuffer_(), modeScheduleUpdated_(false) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ModeScheduleManager::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                                       const CostDesiredTrajectories& costDesiredTrajectory) {
  std::lock_guard<std::mutex> lock(modeScheduleMutex_);
  if (modeScheduleUpdated_) {
    modeScheduleUpdated_ = false;
    swap(modeSchedule_, modeScheduleBuffer_);
  }
  preSolverRunImpl(initTime, finalTime, currentState, costDesiredTrajectory, modeSchedule_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const ModeSchedule& ModeScheduleManager::getModeSchedule() const {
  return modeSchedule_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModeSchedule ModeScheduleManager::getModeScheduleImage() const {
  std::lock_guard<std::mutex> lock(modeScheduleMutex_);
  return modeSchedule_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ModeScheduleManager::setModeSchedule(const ModeSchedule& modeSchedule) {
  std::lock_guard<std::mutex> lock(modeScheduleMutex_);
  modeScheduleUpdated_ = true;
  modeScheduleBuffer_ = modeSchedule;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ModeScheduleManager::setModeSchedule(ModeSchedule&& modeSchedule) {
  std::lock_guard<std::mutex> lock(modeScheduleMutex_);
  modeScheduleUpdated_ = true;
  modeScheduleBuffer_ = std::move(modeSchedule);
}

}  // namespace ocs2
