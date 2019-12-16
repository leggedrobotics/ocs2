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

#ifndef SYSTEMEVENTHANDLER_OCS2_H_
#define SYSTEMEVENTHANDLER_OCS2_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <limits>
#include <memory>
#include <vector>

#include "ocs2_core/integration/EventHandlerBase.h"

namespace ocs2 {

/**
 * System event ID. all values are negative.
 */
enum sys_event_id {
  killIntegration = -1,  //!< killIntegration: kill integration due to an external signal.
  maxCall = -2           //!< maximum number of function calls.
};

/**
 * Specialized event handler for handling toolbox invoked events.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 */
template <int STATE_DIM>
class SystemEventHandler : public EventHandlerBase<STATE_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<SystemEventHandler<STATE_DIM> >;

  using BASE = EventHandlerBase<STATE_DIM>;
  using typename BASE::dynamic_vector_t;
  using typename BASE::scalar_array_t;
  using typename BASE::scalar_t;
  using typename BASE::state_vector_array_t;
  using typename BASE::state_vector_t;

  /**
   * Default constructor
   */
  SystemEventHandler() : maxNumSteps_(std::numeric_limits<int>::max()), eventID_(std::numeric_limits<int>::min()) {}

  /**
   * Default destructor
   */
  virtual ~SystemEventHandler() = default;

  virtual bool checkEvent(const state_vector_t& state, const scalar_t& time) override {
    bool terminateFlag = false;

    if (killIntegration_) {
      terminateFlag = true;
      eventID_ = sys_event_id::killIntegration;
    }

    if (BASE::systemPtr_->getNumFunctionCalls() > maxNumSteps_) {
      terminateFlag = true;
      eventID_ = sys_event_id::maxCall;
    }

    return terminateFlag;
  }

  void reset() override {}

  int handleEvent(state_vector_array_t& stateTrajectory, scalar_array_t& timeTrajectory) override { return eventID_; }

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
  static std::atomic_bool killIntegration_; /*=false*/
  int maxNumSteps_;
  int eventID_;
};

template <int STATE_DIM>
std::atomic_bool SystemEventHandler<STATE_DIM>::killIntegration_(false);

}  // namespace ocs2

#endif /* SYSTEMEVENTHANDLER_OCS2_H_ */
