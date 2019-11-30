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

#ifndef OCS2_OBSERVER_H_
#define OCS2_OBSERVER_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <string>
#include <vector>

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/integration/OdeBase.h"
#include "ocs2_core/integration/SystemEventHandler.h"
#include "ocs2_core/misc/Numerics.h"
#include "ocs2_core/model_data/ModelDataBase.h"

namespace ocs2 {

template <int STATE_DIM>
class IntegratorBase;

/**
 * Observer Class
 * @tparam STATE_DIM
 */
template <int STATE_DIM>
class Observer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using scalar_t = typename Dimensions<STATE_DIM, 0>::scalar_t;
  using scalar_array_t = std::vector<scalar_t>;
  using state_vector_t = Eigen::Matrix<scalar_t, STATE_DIM, 1>;
  using state_vector_array_t = std::vector<state_vector_t, Eigen::aligned_allocator<state_vector_t>>;

  using model_data_t = ModelDataBase;
  using model_data_array_t = model_data_t::array_t;

  /**
   * Constructor
   * @param [in] eventHandler
   */
  explicit Observer(const std::shared_ptr<SystemEventHandler<STATE_DIM>>& eventHandlerPtr = nullptr)
      : eventHandlerPtr_(eventHandlerPtr),
        timeTrajectoryPtr_(nullptr),
        stateTrajectoryPtr_(nullptr),
        modelDataTrajectoryPtr_(nullptr),
        initialCall_(false) {}

  /**
   * Observe function to retrieve the variable of interest.
   * @param [in] x: Current state.
   * @param [in] t: Current time.
   */
  void observe(std::shared_ptr<OdeBase<STATE_DIM>> systemPtr, const state_vector_t& x, const scalar_t& t) {
    // Store data
    if (stateTrajectoryPtr_) {
      stateTrajectoryPtr_->push_back(x);
    }
    if (timeTrajectoryPtr_) {
      timeTrajectoryPtr_->push_back(t);
    }

    // extract model data
    if (modelDataTrajectoryPtr_) {
      // check for initial call
      if (initialCall_) {
        model_data_t* modelDataPtr = systemPtr->beginModelDataPtrIterator()->get();
        modelDataTrajectoryPtr_->emplace_back(*modelDataPtr);
      }
      // TODO(mspieler): Double check initial call handling logic
      initialCall_ = systemPtr->nextModelDataPtrIterator() == systemPtr->beginModelDataPtrIterator();

      // get the model data
      while (systemPtr->nextModelDataPtrIterator() != systemPtr->beginModelDataPtrIterator()) {
        --systemPtr->nextModelDataPtrIterator();
        model_data_t* modelDataPtr = systemPtr->nextModelDataPtrIterator()->get();
        if (modelDataTrajectoryPtr_ && numerics::almost_eq(modelDataPtr->time_, t)) {
          modelDataTrajectoryPtr_->emplace_back(*modelDataPtr);
          systemPtr->nextModelDataPtrIterator() = systemPtr->beginModelDataPtrIterator();
          break;
        }
      }
    }
    systemPtr->nextModelDataPtrIterator() = systemPtr->beginModelDataPtrIterator();

    // Check events
    if (stateTrajectoryPtr_ && timeTrajectoryPtr_ && eventHandlerPtr_ && eventHandlerPtr_->checkEvent(x, t)) {
      // Act on the event
      int eventID = eventHandlerPtr_->handleEvent(*stateTrajectoryPtr_, *timeTrajectoryPtr_);

      switch (eventID) {
        case sys_event_id::killIntegration: {
          throw std::runtime_error("Integration terminated due to an external signal triggered by a program.");
          break;
        }
        case sys_event_id::maxCall: {
          std::string msg = "Integration terminated since the maximum number of function calls is reached. ";
          msg += "State at termination time " + std::to_string(t) + ":\n [";
          for (size_t i = 0; i < x.size() - 1; i++) {
            msg += std::to_string(x(i)) + ", ";
          }
          msg += std::to_string(x(x.size() - 1)) + "]\n";
          throw std::runtime_error(msg);
          break;
        }
        default: { throw static_cast<size_t>(eventID); }
      }
    }
  }

  /**
   * Set state trajectory pointer to observer.
   *
   * @param stateTrajectoryPtr: A pointer to state trajectory.
   */
  void setStateTrajectory(state_vector_array_t* stateTrajectoryPtr) { stateTrajectoryPtr_ = stateTrajectoryPtr; }

  /**
   * Set time trajectory pointer to observer.
   *
   * @param timeTrajectoryPtr: A pointer to time trajectory.
   */
  void setTimeTrajectory(scalar_array_t* timeTrajectoryPtr) { timeTrajectoryPtr_ = timeTrajectoryPtr; }

  /**
   * Sets a pointer to model data trajectory.
   *
   * @param modelDataTrajectoryPtr: A pointer to model data trajectory
   */
  void setModelDataTrajectory(model_data_array_t* modelDataTrajectoryPtr) { modelDataTrajectoryPtr_ = modelDataTrajectoryPtr; }

 private:
  std::shared_ptr<SystemEventHandler<STATE_DIM>> eventHandlerPtr_;

  scalar_array_t* timeTrajectoryPtr_;
  state_vector_array_t* stateTrajectoryPtr_;
  model_data_array_t* modelDataTrajectoryPtr_;

  bool initialCall_;
};

}  // namespace ocs2

#endif /* OCS2OBSERVER_H_ */
