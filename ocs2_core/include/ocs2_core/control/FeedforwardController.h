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

#include <ostream>

#include <ocs2_core/Types.h>
#include <ocs2_core/control/ControllerBase.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2 {

/**
 * FeedforwardController provides a time-dependent control law without state-dependent feedback.
 * Commonly, this is used to wrap around a more general controller and extract only the feedforward portion.
 */
class FeedforwardController final : public ControllerBase {
 public:
  /** Constructor, leaves object uninitialized */
  FeedforwardController() = default;

  /**
   * Constructor initializes all required members of the controller.
   *
   * @param [in] controllerTime: Time stamp array of the controller
   * @param [in] controllerFeedforward: The feedforward control input array.
   */
  FeedforwardController(scalar_array_t controllerTime, vector_array_t controllerFeedforward)
      : timeStamp_(std::move(controllerTime)), uffArray_(std::move(controllerFeedforward)) {}

  /**
   * Constructor to initialize the feedforward input data with a general controller rolled-out along a nominal stateTrajectory
   *
   * @param [in] controllerTime the times for the rollout
   * @param [in] stateTrajectory the states for the rollout
   * @param [in] controller the controller to extract the feedforward controls from during a rollout
   */
  FeedforwardController(const scalar_array_t& controllerTime, const vector_array_t& stateTrajectory, ControllerBase* controller);

  /** Copy constructor */
  FeedforwardController(const FeedforwardController& other);

  /** Move constructor */
  FeedforwardController(FeedforwardController&& other);

  /** Copy assignment (copy and swap idiom) */
  FeedforwardController& operator=(FeedforwardController rhs);

  /** Destructor */
  ~FeedforwardController() override = default;

  /**
   * setController Assign control law
   * @param [in] controllerTime: Time stamp array of the controller
   * @param [in] controllerFeedforward: The feedforward control input array.
   */
  void setController(const scalar_array_t& controllerTime, const vector_array_t& controllerFeedforward);

  vector_t computeInput(scalar_t t, const vector_t& x) override;

  void concatenate(const ControllerBase* nextController, int index, int length) override;

  int size() const override;

  ControllerType getType() const override;

  void clear() override;

  bool empty() const override;

  FeedforwardController* clone() const override;

  void display() const override;

  void flatten(const scalar_array_t& timeArray, const std::vector<std::vector<float>*>& flatArray2) const override;

  static FeedforwardController unFlatten(const scalar_array_t& timeArray, const std::vector<std::vector<float> const*>& flatArray2);

 private:
  void flattenSingle(scalar_t time, std::vector<float>& flatArray) const;

 public:
  scalar_array_t timeStamp_;
  vector_array_t uffArray_;

  friend void swap(FeedforwardController& a, FeedforwardController& b) noexcept;
};

void swap(FeedforwardController& a, FeedforwardController& b) noexcept;

std::ostream& operator<<(std::ostream& out, const FeedforwardController& controller);

}  // namespace ocs2
