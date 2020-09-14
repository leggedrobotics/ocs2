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

#include <atomic>
#include <limits>
#include <memory>
#include <utility>

#include <ocs2_core/Types.h>
#include <ocs2_core/integration/OdeBase.h>

namespace ocs2 {

/**
 * System event ID. all values are negative.
 */
enum sys_event_id {
  killIntegration = -1,  //!< killIntegration: kill integration due to an external signal.
  maxCall = -2           //!< maximum number of function calls.
};

/**
 * Event handler class for ode solvers.
 */
class SystemEventHandler {
 public:
  /** Default constructor */
  SystemEventHandler() = default;

  /** Default destructor */
  virtual ~SystemEventHandler() = default;

  /**
   * Checks whether an event is activated. If true, the method should also return
   * a "Non-Negative" ID which indicates the a unique ID for the active events.
   *
   * @param [in] system: System dynamics
   * @param [in] time: The current time.
   * @param [in] state: The current state vector.
   * @return pair of event flag and eventID
   */
  virtual std::pair<bool, size_t> checkEvent(OdeBase& system, scalar_t time, const vector_t& state);

  /**
   * The operation should be performed if an event is activated.
   *
   * @param [in] system: System dynamics
   * @param [in] time: The current time.
   * @param [in] state: The current state vector.
   */
  void handleEvent(OdeBase& system, scalar_t time, const vector_t& state);

  /** Resets the class. */
  virtual void reset() { killIntegration_ = false; };

 private:
  /** Copy constructor */
  SystemEventHandler(const SystemEventHandler& rhs) : SystemEventHandler() {}

 public:
  std::atomic_bool killIntegration_ = {false};
};

}  // namespace ocs2
