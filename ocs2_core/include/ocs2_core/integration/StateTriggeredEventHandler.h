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

#ifndef STATETRIGGEREDEVENTHANDLER_OCS2_H_
#define STATETRIGGEREDEVENTHANDLER_OCS2_H_

#include "ocs2_core/integration/SystemEventHandler.h"

namespace ocs2 {

template <int STATE_DIM>
class StateTriggeredEventHandler : public SystemEventHandler<STATE_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<StateTriggeredEventHandler<STATE_DIM> >;

  using BASE = SystemEventHandler<STATE_DIM>;
  using scalar_t = typename BASE::scalar_t;
  using scalar_array_t = typename BASE::scalar_array_t;
  using state_vector_t = typename BASE::state_vector_t;
  using state_vector_array_t = typename BASE::state_vector_array_t;
  using dynamic_vector_t = typename BASE::dynamic_vector_t;

  /**
   * Default constructor
   */
  StateTriggeredEventHandler() : BASE(), systemEventHandlerTriggered_(false), triggeredEventSurface_(0) { reset(); }

  /**
   * Default destructor
   */
  virtual ~StateTriggeredEventHandler() = default;

  /**
   * Resets the class.
   */
  void reset() override {
    BASE::reset();
    setEventTimesGuard();
  }

  /**
   * Sets parameters to control event times detection.
   *
   * @param [in] minEventTimeDifference: Minimum accepted time difference between two consecutive events.
   * @param [in] lastEventTriggeredTime: Last Time that an event is triggered.
   * @param [in] lastGuardSurfacesValues: The value of the guard functions at lastEventTriggeredTime.
   */
  virtual void setEventTimesGuard(const scalar_t& minEventTimeDifference = 1e-2,
                                  const scalar_t& lastEventTriggeredTime = std::numeric_limits<scalar_t>::lowest(),
                                  const dynamic_vector_t& lastGuardSurfacesValues = dynamic_vector_t::Zero(0)) {
    if (lastEventTriggeredTime > std::numeric_limits<scalar_t>::lowest() && lastGuardSurfacesValues.size() == 0) {
      throw std::runtime_error(
          "Since the time of the last event is provided, "
          "the value of the guard functions at that time should also be provided.");
    }

    minEventTimeDifference_ = minEventTimeDifference;
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
   * Get miminum time required between events occuring
   *
   * @return The value of minEventTimeDifference
   */

  const scalar_t&  getminEventTimeDifference() const {return minEventTimeDifference_; }

  /**
   * Checks if an event is activated.
   *
   * @param [in] state: Current state vector.
   * @param [in] time: Current time.
   * @return boolean:
   */
  bool checkEvent(const state_vector_t& state, const scalar_t& time) override {
    // SystemEventHandler event
    systemEventHandlerTriggered_ = BASE::checkEvent(state, time);
    if (systemEventHandlerTriggered_) {
      return true;
    }

    //** StateTriggered event **//
    BASE::systemPtr_->computeGuardSurfaces(time, state, guardSurfacesValuesCurrent_);

    bool eventTriggered = false;

    if(time- lastEventTriggeredTime_ > minEventTimeDifference_)
    {
    	for (size_t i = 0; i < guardSurfacesValuesPrevious_.size(); i++) {
    		if (guardSurfacesValuesCurrent_[i] <= 0 && guardSurfacesValuesPrevious_(i) > 0) {
    			eventTriggered = true;
    			triggeredEventSurface_ = i;

    		}
    	}
    }

    if (!eventTriggered) {
      guardSurfacesValuesPrevious_ = guardSurfacesValuesCurrent_;
    }

    return eventTriggered;
  }

  /**
   * The operation should be performed if an event is activated. The method gets references to the time and state
   * trajectories. The current time and state are the last elements of their respective container.
   * The method should also return a "Non-Negative" ID which indicates the a unique ID for the active events.
   * Note that will the negative return values are reserved to handle internal events for the program.
   *
   * @param [out] stateTrajectory: The state trajectory which contains the current state vector as its last element.
   * @param [out] timeTrajectory: The time trajectory which contains the current time as its last element.
   * @retune boolean: A non-negative unique ID for the active events.
   */
  int handleEvent(state_vector_array_t& stateTrajectory, scalar_array_t& timeTrajectory) override {
    // SystemEventHandler event
    if (systemEventHandlerTriggered_) {
      return BASE::handleEvent(stateTrajectory, timeTrajectory);
    }

    // Correcting for the zero crossing
    size_t lastIndex;
    scalar_t zeroCrossingTime;
    state_vector_t zeroCrossingState;
    computeZeroCrossing(stateTrajectory, timeTrajectory, lastIndex, zeroCrossingState, zeroCrossingTime);

    if (lastIndex > 0) {
      timeTrajectory.erase(timeTrajectory.begin() + lastIndex + 1, timeTrajectory.end());
      stateTrajectory.erase(stateTrajectory.begin() + lastIndex + 1, stateTrajectory.end());
    }

    //lastEventTriggeredTime_ = timeTrajectory[lastIndex];
    //guardSurfacesValuesPrevious_.swap(guardSurfacesValuesCurrent_);

    // StateTriggered event
    return triggeredEventSurface_;
  }

  /**
   * Computes the zero crossing.
   *
   * @param [in] stateTrajectory: The state trajectory which contains the current state vector as its last element.
   * @param [in] timeTrajectory: The time trajectory which contains the current time as its last element.
   * @param [out] lastIndex: The first index after crossing.
   * @param [out] zeroCrossingState: State at zero crossing.
   * @param [out] zeroCrossingTime: Time of the zero crossing.
   */
  void computeZeroCrossing(const state_vector_array_t& stateTrajectory, const scalar_array_t& timeTrajectory, size_t& lastIndex,
                           state_vector_t& zeroCrossingState, scalar_t& zeroCrossingTime) {
    if (timeTrajectory.size() == 1) {
      lastIndex = 0;
      zeroCrossingTime = timeTrajectory.front();
      zeroCrossingState = stateTrajectory.front();
    }
    else {
      lastIndex = timeTrajectory.size() - 1;

      if (timeTrajectory[timeTrajectory.size() - 2] - lastEventTriggeredTime_ < minEventTimeDifference_) {
        for (int i = timeTrajectory.size() - 2; i >= 0; i--) {
          BASE::systemPtr_->computeGuardSurfaces(timeTrajectory[i], stateTrajectory[i], guardSurfacesValuesPrevious_);
          if (guardSurfacesValuesPrevious_[triggeredEventSurface_] > 0) {
            break;
          } else {
        	  guardSurfacesValuesCurrent_.swap(guardSurfacesValuesPrevious_);
            lastIndex = i;
          }
        }
      }
    }

    dynamic_vector_t zeroCrossingGuardSurfacesValues;
    BASE::systemPtr_->computeGuardSurfaces(zeroCrossingTime, zeroCrossingState, zeroCrossingGuardSurfacesValues);
  }

 protected:
  bool systemEventHandlerTriggered_;

  size_t triggeredEventSurface_;

  dynamic_vector_t guardSurfacesValuesCurrent_;
  dynamic_vector_t guardSurfacesValuesPrevious_;  // memory

  scalar_t minEventTimeDifference_;
  scalar_t lastEventTriggeredTime_;  // memory
};

}  // namespace ocs2

#endif /* STATETRIGGEREDEVENTHANDLER_OCS2_H_ */
