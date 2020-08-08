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

#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_core/control/LinearController.h>

namespace ocs2 {

/**
 * The base class for all controllers adjustment.
 */
class ControllerAdjustmentBase {
 public:
  /**
   * Default constructor.
   */
  ControllerAdjustmentBase() = default;

  /**
   * Default destructor.
   */
  virtual ~ControllerAdjustmentBase() = default;

  /**
   * Adjust the controller based on the last changes in the logic rules.
   *
   * @param [in] eventTimes: The new event times.
   * @param [in] controllerEventTimes: The control policy stock's event times.
   * @param [out] controllerStock: The controller stock which will be modified.
   */
  virtual void adjustController(const scalar_array_t& eventTimes, const scalar_array_t& controllerEventTimes,
                                std::vector<LinearController>& controllersStock) = 0;
};

}  // namespace ocs2
