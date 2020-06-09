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

namespace ocs2 {

/**
 * This is the base class for initializing the DDP-based algorithms.
 */
class SystemOperatingTrajectoriesBase {
 public:
  /**
   * Default constructor
   */
  SystemOperatingTrajectoriesBase() = default;

  /**
   * Default destructor.
   */
  virtual ~SystemOperatingTrajectoriesBase() = default;

  /**
   * Returns pointer to the class.
   *
   * @return A raw pointer to the class.
   */
  virtual SystemOperatingTrajectoriesBase* clone() const = 0;

  /**
   * Gets the Operating Trajectories of the system in time interval [startTime, finalTime] where there is
   * no intermediate switches except possibly the end time.
   *
   * @param [in] initialState: Initial state.
   * @param [in] startTime: Initial time.
   * @param [in] finalTime: Final time.
   * @param [out] timeTrajectory: Output time stamp trajectory.
   * @param [out] stateTrajectory: Output state trajectory.
   * @param [out] inputTrajectory: Output control input trajectory.
   * @param [in] concatOutput: Whether to concatenate the output to the input trajectories or override.
   */
  virtual void getSystemOperatingTrajectories(const vector_t& initialState, scalar_t startTime, scalar_t finalTime,
                                              scalar_array_t& timeTrajectory, vector_array_t& stateTrajectory,
                                              vector_array_t& inputTrajectory, bool concatOutput) = 0;
};

}  // namespace ocs2
