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

#include "ocs2_oc/synchronized_module/ReferenceManager.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ReferenceManager::ReferenceManager(TargetTrajectories initialTargetTrajectories, ModeSchedule initialModeSchedule)
    : targetTrajectories_(std::move(initialTargetTrajectories)), modeSchedule_(std::move(initialModeSchedule)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ReferenceManager::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState) {
  std::lock_guard<std::mutex> lock(dataMutex_);

  if (modeScheduleUpdated_) {
    swap(modeSchedule_, modeScheduleBuffer_);
    modeScheduleUpdated_ = false;
  }

  if (targetTrajectoriesUpdated_) {
    swap(targetTrajectories_, targetTrajectoriesBuffer_);
    targetTrajectoriesUpdated_ = false;
  }

  modifyReferences(initTime, finalTime, initState, targetTrajectories_, modeSchedule_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModeSchedule ReferenceManager::getModeScheduleImage() const {
  std::lock_guard<std::mutex> lock(dataMutex_);
  return modeSchedule_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TargetTrajectories ReferenceManager::getTargetTrajectoriesImage() const {
  std::lock_guard<std::mutex> lock(dataMutex_);
  return targetTrajectories_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ReferenceManager::setModeSchedule(const ModeSchedule& modeSchedule) {
  std::lock_guard<std::mutex> lock(dataMutex_);
  modeScheduleBuffer_ = modeSchedule;
  modeScheduleUpdated_ = true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ReferenceManager::setModeSchedule(ModeSchedule&& modeSchedule) {
  std::lock_guard<std::mutex> lock(dataMutex_);
  swap(modeScheduleBuffer_, modeSchedule);
  modeScheduleUpdated_ = true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ReferenceManager::setTargetTrajectories(const TargetTrajectories& targetTrajectories) {
  std::lock_guard<std::mutex> lock(dataMutex_);
  targetTrajectoriesBuffer_ = targetTrajectories;
  targetTrajectoriesUpdated_ = true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ReferenceManager::setTargetTrajectories(TargetTrajectories&& targetTrajectories) {
  std::lock_guard<std::mutex> lock(dataMutex_);
  swap(targetTrajectoriesBuffer_, targetTrajectories);
  targetTrajectoriesUpdated_ = true;
}

}  // namespace ocs2
