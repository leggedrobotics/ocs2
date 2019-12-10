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

#include "ocs2_core/integration/SystemEventHandler.h"

namespace ocs2 {

template <int STATE_DIM>
class StateTriggeredEventHandler final : public SystemEventHandler<STATE_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = SystemEventHandler<STATE_DIM>;
  using typename BASE::dynamic_vector_t;
  using typename BASE::scalar_t;
  using typename BASE::state_vector_t;

  /**
   * Constructor
   *
   * @param [in] minEventTimeDifference: Minimum accepted time difference between two consecutive events.
   */
  explicit StateTriggeredEventHandler(scalar_t minEventTimeDifference)
      : BASE(), minEventTimeDifference_(std::move(minEventTimeDifference)) {
    reset();
  }

  /**
   * Default destructor
   */
  ~StateTriggeredEventHandler() override = default;

  bool checkEvent(scalar_t time, const state_vector_t& state, size_t& eventID) override {
    // StateTriggered event
    BASE::systemPtr_->computeGuardSurfaces(time, state, guardSurfacesValuesCurrent_);

    bool eventTriggered = false;
    if (time - lastEventTriggeredTime_ > minEventTimeDifference_) {
      for (size_t i = 0; i < guardSurfacesValuesPrevious_.size(); i++) {
        if (guardSurfacesValuesCurrent_(i) <= 0 && guardSurfacesValuesPrevious_(i) > 0) {
          eventTriggered = true;
          eventID = i;
        }
      }
    }

    // update guard surfaces if event is not triggered
    if (!eventTriggered) {
      guardSurfacesValuesPrevious_ = guardSurfacesValuesCurrent_;
    }

    return eventTriggered;
  }

  /**
   * Sets parameters to control event times detection.
   *
   * @param [in] lastEventTriggeredTime: Last Time that an event is triggered.
   * @param [in] lastGuardSurfacesValues: The value of the guard functions at lastEventTriggeredTime.
   */
  void setLastEvent(scalar_t lastEventTriggeredTime, const dynamic_vector_t& lastGuardSurfacesValues) {
    lastEventTriggeredTime_ = lastEventTriggeredTime;
    guardSurfacesValuesPrevious_ = lastGuardSurfacesValues;
  }

  /**
   * Gets the value of the guard surfaces.
   *
   * @return The value of the guard surfaces.
   */
  const dynamic_vector_t& getGuardSurfacesValues() const { return guardSurfacesValuesPrevious_; }

  /**
   * Gets the minimum acceptable time in between two consecutive events.
   *
   * @return The value of minEventTimeDifference_
   */
  scalar_t getminEventTimeDifference() const { return minEventTimeDifference_; }

  /**
   * Resets the class.
   */
  void reset() {
    lastEventTriggeredTime_ = std::numeric_limits<scalar_t>::lowest();
    guardSurfacesValuesPrevious_.setZero(0);
  }

 protected:
  scalar_t minEventTimeDifference_;

  dynamic_vector_t guardSurfacesValuesCurrent_;
  dynamic_vector_t guardSurfacesValuesPrevious_;  // memory
  scalar_t lastEventTriggeredTime_;               // memory
};

}  // namespace ocs2
