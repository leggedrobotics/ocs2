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
#include <algorithm>
#include <iterator>
#include <type_traits>
#include <vector>

#include "ocs2_core/initialization/SystemOperatingTrajectoriesBase.h"
#include "ocs2_core/misc/LinearInterpolation.h"
#include "ocs2_core/misc/Numerics.h"

namespace ocs2 {

/**
 * This is base class for initializing the SLQ-based algorithms.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class OperatingPoints : public SystemOperatingTrajectoriesBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using DIMENSIONS = typename SystemOperatingTrajectoriesBase<STATE_DIM, INPUT_DIM>::DIMENSIONS;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using size_array_t = typename DIMENSIONS::size_array_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;

  /**
   * Constructor
   *
   * @param [in] stateOperatingPoint: The state operating point.
   * @param [in] inputOperatingPoint: The input operating point.
   */
  OperatingPoints(const state_vector_t& stateOperatingPoint, const input_vector_t& inputOperatingPoint)
      : timeTrajectory_(1, 0.0), stateTrajectory_(1, stateOperatingPoint), inputTrajectory_(1, inputOperatingPoint) {}

  /**
   * Constructor
   *
   * @param [in] timeTrajectory: The time stamp of the operating trajectories.
   * @param [in] stateTrajectory: The state operating trajectory.
   * @param [in] inputTrajectory: The input operating trajectory.
   */
  OperatingPoints(scalar_array_t timeTrajectory, const state_vector_array_t& stateTrajectory, const input_vector_array_t& inputTrajectory)
      : timeTrajectory_(std::move(timeTrajectory)), stateTrajectory_(stateTrajectory), inputTrajectory_(inputTrajectory) {}

  ~OperatingPoints() override = default;

  OperatingPoints<STATE_DIM, INPUT_DIM>* clone() const override { return new OperatingPoints(*this); }

  void getSystemOperatingTrajectories(const state_vector_t& initialState, const scalar_t& startTime, const scalar_t& finalTime,
                                      scalar_array_t& timeTrajectory, state_vector_array_t& stateTrajectory,
                                      input_vector_array_t& inputTrajectory, bool concatOutput = false) override {
    if (!concatOutput) {
      timeTrajectory.clear();
      stateTrajectory.clear();
      inputTrajectory.clear();
    }

    getSystemOperatingTrajectoriesImpl(startTime, finalTime, timeTrajectory, stateTrajectory, inputTrajectory);
  }

 private:
  /**
   * Gets the Operating Trajectories of the system in time interval [startTime, finalTime] where there is no intermediate switches.
   *
   * @param [in] startTime: Initial time.
   * @param [in] finalTime: Final time.
   * @param [out] timeTrajectory: Output time stamp trajectory.
   * @param [out] stateTrajectory: Output state trajectory.
   * @param [out] inputTrajectory: Output control input trajectory.
   */
  void getSystemOperatingTrajectoriesImpl(scalar_t startTime, scalar_t finalTime, scalar_array_t& timeTrajectory,
                                          state_vector_array_t& stateTrajectory, input_vector_array_t& inputTrajectory) const {
    const auto initIndexAlpha = EigenLinearInterpolation<state_vector_t>::timeSegment(startTime, &timeTrajectory_);
    const auto finalindexAlpha = EigenLinearInterpolation<state_vector_t>::timeSegment(finalTime, &timeTrajectory_);

    state_vector_t x0;
    EigenLinearInterpolation<state_vector_t>::interpolate(initIndexAlpha, x0, &stateTrajectory_);
    input_vector_t u0;
    EigenLinearInterpolation<input_vector_t>::interpolate(initIndexAlpha, u0, &inputTrajectory_);
    timeTrajectory.push_back(startTime);
    stateTrajectory.emplace_back(x0);
    inputTrajectory.emplace_back(u0);

    // if the time interval is not empty
    if (!numerics::almost_eq(startTime, finalTime)) {
      const auto beginIndex = initIndexAlpha.first + 1;
      const auto lastIndex = finalindexAlpha.first + 1;
      std::copy(timeTrajectory_.begin() + beginIndex, timeTrajectory_.begin() + lastIndex, std::back_inserter(timeTrajectory));
      std::copy(stateTrajectory_.begin() + beginIndex, stateTrajectory_.begin() + lastIndex, std::back_inserter(stateTrajectory));
      std::copy(inputTrajectory_.begin() + beginIndex, inputTrajectory_.begin() + lastIndex, std::back_inserter(inputTrajectory));

      state_vector_t xf;
      EigenLinearInterpolation<state_vector_t>::interpolate(finalindexAlpha, xf, &stateTrajectory_);
      input_vector_t uf;
      EigenLinearInterpolation<input_vector_t>::interpolate(finalindexAlpha, uf, &inputTrajectory_);
      timeTrajectory.push_back(finalTime);
      stateTrajectory.emplace_back(xf);
      inputTrajectory.emplace_back(uf);
    }
  }

  scalar_array_t timeTrajectory_;
  state_vector_array_t stateTrajectory_;
  input_vector_array_t inputTrajectory_;
};

}  // namespace ocs2
