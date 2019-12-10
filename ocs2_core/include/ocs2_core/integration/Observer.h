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
#include <string>
#include <vector>

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/integration/OdeBase.h"
#include "ocs2_core/integration/SystemEventHandler.h"
#include "ocs2_core/misc/Numerics.h"
#include "ocs2_core/model_data/ModelDataBase.h"

namespace ocs2 {

/**
 * Observer Class
 * @tparam STATE_DIM
 *
 * The Observer class stores data in given containers and handles events during integration.
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

  using observer_callback_t = std::function<void(const state_vector_t& x, scalar_t t)>;

  /**
   * Constructor
   * @param [in] eventHandler
   */
  explicit Observer(const std::shared_ptr<OdeBase<STATE_DIM>> systemPtr,
                    const std::shared_ptr<SystemEventHandler<STATE_DIM>> eventHandlerPtr = nullptr)
      : systemPtr_(std::move(systemPtr)), eventHandlerPtr_(std::move(eventHandlerPtr)), initialCall_(false) {}

  /**
   * Make observer callback function for integration
   * @param [out] timeTraj: Time of each trajectory point
   * @param [out] stateTraj: State trajectory
   * @param [out] modelDataTraj: Model data trajectory
   */
  observer_callback_t getCallback(scalar_array_t* timeTraj = nullptr, state_vector_array_t* stateTraj = nullptr,
                                  model_data_array_t* modelDataTraj = nullptr) {
    return [=](const state_vector_t& x, scalar_t t) { observe(x, t, timeTraj, stateTraj, modelDataTraj); };
  }

 private:
  /**
   * Observe function to retrieve the variable of interest.
   * @param [in] x: Current state.
   * @param [in] t: Current time.
   */
  void observe(const state_vector_t& x, const scalar_t& t, scalar_array_t* timeTraj, state_vector_array_t* stateTraj,
               model_data_array_t* modelDataTraj) {
    // Store data
    if (stateTraj) {
      stateTraj->push_back(x);
    }
    if (timeTraj) {
      timeTraj->push_back(t);
    }

    if (systemPtr_) {
      // extract model data
      if (modelDataTraj) {
        // check for initial call
        if (systemPtr_->nextModelDataPtrIterator() == systemPtr_->beginModelDataPtrIterator()) {
          // do nothing, model data not yet available.
          initialCall_ = true;
        } else if (initialCall_) {
          // this is second observer call, retreive model data from initial call.
          model_data_t* modelDataPtr = systemPtr_->beginModelDataPtrIterator()->get();
          modelDataTraj->emplace_back(*modelDataPtr);
          initialCall_ = false;
        }
        // get model data from current timestep
        while (systemPtr_->nextModelDataPtrIterator() != systemPtr_->beginModelDataPtrIterator()) {
          --systemPtr_->nextModelDataPtrIterator();
          model_data_t* modelDataPtr = systemPtr_->nextModelDataPtrIterator()->get();
          if (numerics::almost_eq(modelDataPtr->time_, t)) {
            modelDataTraj->emplace_back(*modelDataPtr);
            break;
          }
        }
      }
      // reset modelDataPtrArray write position
      systemPtr_->nextModelDataPtrIterator() = systemPtr_->beginModelDataPtrIterator();
    }
  }

  std::shared_ptr<OdeBase<STATE_DIM>> systemPtr_;
  std::shared_ptr<SystemEventHandler<STATE_DIM>> eventHandlerPtr_;

  bool initialCall_;
};

}  // namespace ocs2
