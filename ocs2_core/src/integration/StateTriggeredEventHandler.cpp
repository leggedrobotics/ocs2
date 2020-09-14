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

#include <ocs2_core/integration/StateTriggeredEventHandler.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateTriggeredEventHandler::StateTriggeredEventHandler(scalar_t minEventTimeDifference)
    : SystemEventHandler(), minEventTimeDifference_(minEventTimeDifference) {
  reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<bool, size_t> StateTriggeredEventHandler::checkEvent(OdeBase& system, scalar_t time, const vector_t& state) {
  // state-triggered event
  guardSurfacesValuesCurrent_ = system.computeGuardSurfaces(time, state);

  size_t eventID = 0;
  bool eventTriggered = false;
  if (time - lastEventTriggeredTime_ > minEventTimeDifference_) {
    for (size_t i = 0; i < guardSurfacesValuesPrevious_.size(); i++) {
      if (guardSurfacesValuesCurrent_(i) <= 0 && guardSurfacesValuesPrevious_(i) > 0) {
        eventID = i;
        eventTriggered = true;
      }
    }
  }

  // update guard surfaces if event is not triggered
  if (!eventTriggered) {
    guardSurfacesValuesPrevious_ = guardSurfacesValuesCurrent_;
  }

  return {eventTriggered, eventID};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateTriggeredEventHandler::setLastEvent(scalar_t lastEventTriggeredTime, const vector_t& lastGuardSurfacesValues) {
  lastEventTriggeredTime_ = lastEventTriggeredTime;
  guardSurfacesValuesPrevious_ = lastGuardSurfacesValues;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const vector_t& StateTriggeredEventHandler::getGuardSurfacesValues() const {
  return guardSurfacesValuesPrevious_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t StateTriggeredEventHandler::getminEventTimeDifference() const {
  return minEventTimeDifference_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateTriggeredEventHandler::reset() {
  SystemEventHandler::reset();
  lastEventTriggeredTime_ = std::numeric_limits<scalar_t>::lowest();
  guardSurfacesValuesPrevious_.setZero(0);
}

}  // namespace ocs2
