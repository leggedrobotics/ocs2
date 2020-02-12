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

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <exception>
#include <memory>
#include <utility>
#include <vector>

#include <ocs2_core/Dimensions.h>
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
 *
 * @tparam STATE_DIM: Dimension of the state space.
 */
template <int STATE_DIM>
class SystemEventHandler {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using DIMENSIONS = Dimensions<STATE_DIM, 0>;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using dynamic_vector_t = typename DIMENSIONS::dynamic_vector_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;

  using system_t = OdeBase<STATE_DIM>;

  /**
   * Default constructor
   */
  SystemEventHandler() = default;

  /**
   * Default destructor
   */
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
  virtual std::pair<bool, size_t> checkEvent(system_t& system, scalar_t time, const state_vector_t& state) { return {false, 0}; }

  /**
   * The operation should be performed if an event is activated.
   *
   * @param [in] system: System dynamics
   * @param [in] time: The current time.
   * @param [in] state: The current state vector.
   */
  void handleEvent(system_t& system, scalar_t time, const state_vector_t& state) {
    // kill integration is triggered
    if (killIntegration_) {
      throw std::runtime_error("Integration terminated due to an external signal triggered by a program.");
    }

    // max number of function calls
    if (system.getNumFunctionCalls() > maxNumSteps_) {
      std::string msg = "Integration terminated since the maximum number of function calls is reached. ";
      msg += "State at termination time " + std::to_string(time) + ":\n [";
      for (size_t i = 0; i < state.size() - 1; i++) {
        msg += std::to_string(state(i)) + ", ";
      }
      msg += std::to_string(state(state.size() - 1)) + "]\n";
      throw std::runtime_error(msg);
    }

    // derived class events
    size_t eventID;
    bool event;
    std::tie(event, eventID) = this->checkEvent(system, time, state);
    if (event) {
      throw eventID;
    }
  }

  /**
   * Resets the class.
   */
  virtual void reset() {}

  /**
   * Sets the maximum number of integration points per a second for ode solvers.
   *
   * @param [in] maxNumSteps: maximum number of integration points
   */
  void setMaxNumSteps(int maxNumSteps) { maxNumSteps_ = maxNumSteps; }

  /**
   * Activate KillIntegrationEvent.
   */
  static void activateKillIntegration() { killIntegration_ = true; }

  /**
   * Deactivate KillIntegrationEvent.
   */
  static void deactivateKillIntegration() { killIntegration_ = false; }

 protected:
  static std::atomic<bool> killIntegration_; /*=false*/
  int maxNumSteps_ = std::numeric_limits<int>::max();
};

template <int STATE_DIM>
std::atomic<bool> SystemEventHandler<STATE_DIM>::killIntegration_(false);

}  // namespace ocs2
