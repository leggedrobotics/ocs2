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

#include <ocs2_core/integration/SystemEventHandler.h>

namespace ocs2 {

/**
 * State triggered event handler class for ode solvers.
 */
class StateTriggeredEventHandler final : public SystemEventHandler {
 public:
  /**
   * Constructor
   *
   * @param [in] minEventTimeDifference: Minimum accepted time difference between two consecutive events.
   */
  explicit StateTriggeredEventHandler(scalar_t minEventTimeDifference);

  /**
   * Default destructor
   */
  ~StateTriggeredEventHandler() override = default;

  /**
   * Checks whether an event is activated. If true, the method should also return
   * a "Non-Negative" ID which indicates the a unique ID for the active events.
   *
   * @param [in] system: System dynamics
   * @param [in] time: The current time.
   * @param [in] state: The current state vector.
   * @return pair of event flag and eventID
   */
  std::pair<bool, size_t> checkEvent(OdeBase& system, scalar_t time, const vector_t& state) override;

  /**
   * Sets parameters to control event times detection.
   *
   * @param [in] lastEventTriggeredTime: Last Time that an event is triggered.
   * @param [in] lastGuardSurfacesValues: The value of the guard functions at lastEventTriggeredTime.
   */
  void setLastEvent(scalar_t lastEventTriggeredTime, const vector_t& lastGuardSurfacesValues);

  /**
   * Gets the value of the guard surfaces.
   *
   * @return The value of the guard surfaces.
   */
  const vector_t& getGuardSurfacesValues() const;

  /**
   * Gets the minimum acceptable time in between two consecutive events.
   *
   * @return The value of minEventTimeDifference_
   */
  scalar_t getminEventTimeDifference() const;

  /**
   * Resets the class.
   */
  void reset() override;

 protected:
  scalar_t minEventTimeDifference_;
  vector_t guardSurfacesValuesCurrent_;
  vector_t guardSurfacesValuesPrevious_;
  scalar_t lastEventTriggeredTime_ = std::numeric_limits<scalar_t>::lowest();
};

}  // namespace ocs2
