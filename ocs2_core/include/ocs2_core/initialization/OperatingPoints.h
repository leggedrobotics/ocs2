/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2 {

/**
 * This class initializes the solver based on operating trajectories or single points of state and input.
 */
class OperatingPoints final : public Initializer {
 public:
  /**
   * Constructor
   * @param [in] stateOperatingPoint: A single state operating point.
   * @param [in] inputOperatingPoint: A single operating point.
   */
  OperatingPoints(const vector_t& stateOperatingPoint, const vector_t& inputOperatingPoint)
      : Initializer(Eigen::Dynamic),
        timeTrajectory_(1, 0.0),
        stateTrajectory_(1, stateOperatingPoint),
        inputTrajectory_(1, inputOperatingPoint) {}

  /**
   * Constructor
   * @param [in] timeTrajectory: The time stamp of the operating trajectories.
   * @param [in] stateTrajectory: The state operating trajectory.
   * @param [in] inputTrajectory: The input operating trajectory.
   */
  OperatingPoints(scalar_array_t timeTrajectory, vector_array_t stateTrajectory, vector_array_t inputTrajectory)
      : Initializer(Eigen::Dynamic),
        timeTrajectory_(std::move(timeTrajectory)),
        stateTrajectory_(std::move(stateTrajectory)),
        inputTrajectory_(std::move(inputTrajectory)) {}

  /** Destructor */
  ~OperatingPoints() override = default;

  OperatingPoints* clone() const override { return new OperatingPoints(*this); }

  void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override {
    input = LinearInterpolation::interpolate(time, timeTrajectory_, inputTrajectory_);
    nextState = LinearInterpolation::interpolate(nextTime, timeTrajectory_, stateTrajectory_);
  }

 private:
  /** Copy constructor */
  OperatingPoints(const OperatingPoints& other) = default;

  const scalar_array_t timeTrajectory_;
  const vector_array_t stateTrajectory_;
  const vector_array_t inputTrajectory_;
};

}  // namespace ocs2
