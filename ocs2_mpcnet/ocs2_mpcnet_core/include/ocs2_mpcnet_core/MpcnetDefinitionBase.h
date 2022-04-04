/******************************************************************************
Copyright (c) 2022, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_core/reference/TargetTrajectories.h>

namespace ocs2 {
namespace mpcnet {

/**
 * Base class for MPC-Net definitions.
 */
class MpcnetDefinitionBase {
 public:
  /**
   * Default constructor.
   */
  MpcnetDefinitionBase() = default;

  /**
   * Default destructor.
   */
  virtual ~MpcnetDefinitionBase() = default;

  /**
   * Deleted copy constructor.
   */
  MpcnetDefinitionBase(const MpcnetDefinitionBase&) = delete;

  /**
   * Deleted copy assignment.
   */
  MpcnetDefinitionBase& operator=(const MpcnetDefinitionBase&) = delete;

  /**
   * Get the generalized time.
   * @param[in] t : Absolute time.
   * @param[in] modeSchedule : Mode schedule.
   * @return The generalized time.
   */
  virtual vector_t getGeneralizedTime(scalar_t t, const ModeSchedule& modeSchedule) = 0;

  /**
   * Get the relative state.
   * @param[in] t : Absolute time.
   * @param[in] x : Robot state.
   * @param[in] targetTrajectories : Target trajectories.
   * @return The relative state.
   */
  virtual vector_t getRelativeState(scalar_t t, const vector_t& x, const TargetTrajectories& targetTrajectories) = 0;

  /**
   * Get the input transformation.
   * @param[in] t : Absolute time.
   * @param[in] x : Robot state.
   * @return The input transformation.
   */
  virtual matrix_t getInputTransformation(scalar_t t, const vector_t& x) = 0;

  /**
   * Check if a state is valid.
   * @param [in] x : State.
   * @return True if valid.
   */
  virtual bool validState(const vector_t& x) = 0;
};

}  // namespace mpcnet
}  // namespace ocs2
