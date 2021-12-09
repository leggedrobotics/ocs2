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

#include <ocs2_core/Types.h>
#include <ocs2_core/control/ControllerBase.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2 {

/**
 * LinearController implements a time and state dependent controller of the
 * form u[x,t] = k[t] * x + uff[t]
 */
class LinearController final : public ControllerBase {
 public:
  /** Constructor, leaves object uninitialized */
  LinearController() = default;

  /**
   * @brief Constructor initializes all required members of the controller.
   *
   * @param [in] controllerTime: Time stamp array of the controller
   * @param [in] controllerBias: The bias array.
   * @param [in] controllerGain: The feedback gain array.
   */
  LinearController(scalar_array_t controllerTime, vector_array_t controllerBias, matrix_array_t controllerGain)
      : timeStamp_(std::move(controllerTime)), biasArray_(std::move(controllerBias)), gainArray_(std::move(controllerGain)) {}

  /** Copy constructor */
  LinearController(const LinearController& other);

  /** Move constructor */
  LinearController(LinearController&& other);

  /** Copy assignment (copy and swap idiom) */
  LinearController& operator=(LinearController rhs);

  /** Destructor */
  ~LinearController() override = default;

  /** Clone */
  LinearController* clone() const override;

  /**
   * @brief setController Assign control law
   * @param [in] controllerTime: Time stamp array of the controller
   * @param [in] controllerBias: The bias array.
   * @param [in] controllerGain: The feedback gain array.
   */
  void setController(const scalar_array_t& controllerTime, const vector_array_t& controllerBias, const matrix_array_t& controllerGain);

  vector_t computeInput(scalar_t t, const vector_t& x) override;

  void concatenate(const ControllerBase* nextController, int index, int length) override;

  int size() const override;

  ControllerType getType() const override;

  void clear() override;

  bool empty() const override;

  void display() const override;

  void getFeedbackGain(scalar_t time, matrix_t& gain) const;

  void getBias(scalar_t time, vector_t& bias) const;

  scalar_array_t controllerEventTimes() const override;

  void flatten(const scalar_array_t& timeArray, const std::vector<std::vector<float>*>& flatArray2) const override;

  static LinearController unFlatten(const size_array_t& stateDim, const size_array_t& inputDim, const scalar_array_t& timeArray,
                                    const std::vector<std::vector<float> const*>& flatArray2);

 private:
  void flattenSingle(scalar_t time, std::vector<float>& flatArray) const;

 public:
  scalar_array_t timeStamp_;
  vector_array_t biasArray_;
  vector_array_t deltaBiasArray_;
  matrix_array_t gainArray_;

  friend void swap(LinearController& a, LinearController& b) noexcept;
};

void swap(LinearController& a, LinearController& b) noexcept;

std::ostream& operator<<(std::ostream& out, const LinearController& controller);

}  // namespace ocs2
