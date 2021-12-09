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

#include <ocs2_core/control/ControllerBase.h>
#include <ocs2_core/control/LinearController.h>

namespace ocs2 {

class StateBasedLinearController final : public ControllerBase {
 public:
  /** Default constructor */
  StateBasedLinearController() = default;

  /** Default destructor */
  ~StateBasedLinearController() override = default;

  /**
   * Sets the provided controller pointer which provides the input signal
   * At the same time the eventtimes of the designed controller are computed
   *
   * @param[in] ctrlPtr: pointer to the provided controller
   */
  void setController(ControllerBase* ctrlPtr);

  /**
   * Computes the control input based on the trajectory spreading scheme.
   *
   * @param [in] t: current time at which input is requested
   * @param [in] x: current state at which input is requested
   * @param [in] ctrlEventTimes: array containing eventTimes around which the controller was designed
   * @param [in] ctrlPtr: Pointer to the actual controller
   * @retrun control input vector
   */
  static vector_t computeTrajectorySpreadingInput(scalar_t t, const vector_t& x, const scalar_array_t& ctrlEventTimes,
                                                  ControllerBase* ctrlPtr);

  vector_t computeInput(scalar_t t, const vector_t& x) override;

  void concatenate(const ControllerBase* nextController, int index, int length) override;

  int size() const override;

  ControllerType getType() const override;

  void clear() override;

  bool empty() const override;

  void display() const override;

  StateBasedLinearController* clone() const override;

 private:
  ControllerBase* ctrlPtr_ = nullptr;
  scalar_array_t ctrlEventTimes_{0};
};

}  // namespace ocs2
