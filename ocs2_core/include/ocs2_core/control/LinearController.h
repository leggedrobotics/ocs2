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
  using array_t = std::vector<LinearController>;

  /**
   * @brief Default constructor leaves object uninitialized
   */
  LinearController() = default;

  /**
   * @brief Constructor initializes all required members of the controller.
   *
   * @param [in] controllerTime: Time stamp array of the controller
   * @param [in] controllerBias: The bias array.
   * @param [in] controllerGain: The feedback gain array.
   */
  LinearController(const scalar_array_t& controllerTime, const vector_array_t& controllerBias, const matrix_array_t& controllerGain);
  /**
   * @brief Copy constructor
   * @param other LinearController object to copy from
   */
  LinearController(const LinearController& other);

  /**
   * @brief LinearController move constructor -- not implemented for now
   * @param other LinearController object to move from
   * @todo Implement
   */
  LinearController(LinearController&& other) = delete;

  /**
   * @brief Copy assignment (copy and swap idiom)
   * @param other LinearController object to assign from
   */
  LinearController& operator=(LinearController other);

  /**
   * @brief Move assignment -- not implemented for now
   * @param other LinearController object to assign from
   * @todo Implement
   */
  LinearController& operator=(LinearController&& other) = delete;

  /**
   * @brief Destructor
   */
  ~LinearController() override = default;

  LinearController* clone() const override;

  /**
   * @brief setController Assign control law
   * @param [in] controllerTime: Time stamp array of the controller
   * @param [in] controllerBias: The bias array.
   * @param [in] controllerGain: The feedback gain array.
   */
  void setController(const scalar_array_t& controllerTime, const vector_array_t& controllerBias, const matrix_array_t& controllerGain);

  vector_t computeInput(const scalar_t& t, const vector_t& x) override;

  void flatten(const scalar_array_t& timeArray, const std::vector<float_array_t*>& flatArray2) const override;

  void flattenSingle(scalar_t time, float_array_t& flatArray) const;

  void unFlatten(const scalar_array_t& timeArray, const std::vector<float_array_t const*>& flatArray2) override;

  void concatenate(const ControllerBase* nextController, int index, int length) override;

  int size() const override;

  ControllerType getType() const override;

  void clear() override;

  void setZero() override;

  bool empty() const override;

  void display() const override;

  virtual void swap(LinearController& other);

  void getFeedbackGain(scalar_t time, matrix_t& gain) const;

  void getBias(scalar_t time, vector_t& bias) const;

  scalar_array_t controllerEventTimes() const override;

 public:
  scalar_array_t timeStamp_;
  vector_array_t biasArray_;
  vector_array_t deltaBiasArray_;
  matrix_array_t gainArray_;
};

void swap(LinearController& a, LinearController& b);

std::ostream& operator<<(std::ostream& out, const LinearController& controller);

}  // namespace ocs2
