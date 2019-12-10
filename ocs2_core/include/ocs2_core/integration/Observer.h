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

  /**
   * Constructor
   * @param [out] stateTraj: State trajectory
   * @param [out] timeTraj: Time of each trajectory point
   * @param [out] modelDataTraj: Model data trajectory
   */
  explicit Observer(state_vector_array_t* stateTraj = nullptr, scalar_array_t* timeTraj = nullptr,
                    model_data_array_t* modelDataTraj = nullptr)
      : timeTrajectory_(timeTraj), stateTrajectory_(stateTraj), modelDataTrajectory_(modelDataTraj), initialCall_(false) {}

  /**
   * Observe function to retrieve the variable of interest.
   * @param [in] system: system dynamics object.
   * @param [in] t: Current time.
   * @param [in] x: Current state.
   */
  void observe(OdeBase<STATE_DIM>& system, const state_vector_t& x, const scalar_t& t) {
    // Store data
    if (stateTrajectory_) {
      stateTrajectory_->push_back(x);
    }
    if (timeTrajectory_) {
      timeTrajectory_->push_back(t);
    }

    // extract model data
    if (modelDataTrajectory_) {
      // check for initial call
      if (system.nextModelDataPtrIterator() == system.beginModelDataPtrIterator()) {
        // do nothing, model data not yet available.
        initialCall_ = true;
      } else if (initialCall_) {
        // this is second observer call, retreive model data from initial call.
        model_data_t* modelDataPtr = system.beginModelDataPtrIterator()->get();
        modelDataTrajectory_->emplace_back(*modelDataPtr);
        initialCall_ = false;
      }
      // get model data from current timestep
      while (system.nextModelDataPtrIterator() != system.beginModelDataPtrIterator()) {
        --system.nextModelDataPtrIterator();
        model_data_t* modelDataPtr = system.nextModelDataPtrIterator()->get();
        if (numerics::almost_eq(modelDataPtr->time_, t)) {
          modelDataTrajectory_->emplace_back(*modelDataPtr);
          break;
        }
      }
      // reset modelDataPtrArray write position
      system.nextModelDataPtrIterator() = system.beginModelDataPtrIterator();
    }
  }

 private:
  bool initialCall_;

  state_vector_array_t* stateTrajectory_;
  scalar_array_t* timeTrajectory_;
  model_data_array_t* modelDataTrajectory_;
};

}  // namespace ocs2
