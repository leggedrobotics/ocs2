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

#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_mpcnet_core/MpcnetDefinitionBase.h>

namespace ocs2 {
namespace legged_robot {

/**
 * MPC-Net definitions for legged robot.
 */
class LeggedRobotMpcnetDefinition final : public ocs2::mpcnet::MpcnetDefinitionBase {
 public:
  /**
   * Constructor.
   * @param [in] leggedRobotInterface : Legged robot interface.
   */
  LeggedRobotMpcnetDefinition(const LeggedRobotInterface& leggedRobotInterface)
      : defaultState_(leggedRobotInterface.getInitialState()), centroidalModelInfo_(leggedRobotInterface.getCentroidalModelInfo()) {}

  /**
   * Default destructor.
   */
  ~LeggedRobotMpcnetDefinition() override = default;

  /**
   * @see MpcnetDefinitionBase::getObservation
   */
  vector_t getObservation(scalar_t t, const vector_t& x, const ModeSchedule& modeSchedule,
                          const TargetTrajectories& targetTrajectories) override;

  /**
   * @see MpcnetDefinitionBase::getActionTransformation
   */
  std::pair<matrix_t, vector_t> getActionTransformation(scalar_t t, const vector_t& x, const ModeSchedule& modeSchedule,
                                                        const TargetTrajectories& targetTrajectories) override;

  /**
   * @see MpcnetDefinitionBase::isValid
   */
  bool isValid(scalar_t t, const vector_t& x, const ModeSchedule& modeSchedule, const TargetTrajectories& targetTrajectories) override;

 private:
  const scalar_t allowedHeightDeviation_ = 0.2;
  const scalar_t allowedPitchDeviation_ = 30.0 * M_PI / 180.0;
  const scalar_t allowedRollDeviation_ = 30.0 * M_PI / 180.0;
  const vector_t defaultState_;
  const CentroidalModelInfo centroidalModelInfo_;
};

}  // namespace legged_robot
}  // namespace ocs2
