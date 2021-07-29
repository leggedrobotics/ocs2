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

// ocs2
#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

// ballbot
#include <ocs2_ballbot/BallbotParameters.h>
#include "ocs2_ballbot/definitions.h"

namespace ocs2 {
namespace ballbot {

/**
 * BallbotSystemDynamics class
 * This class implements the dynamics for the ballbot.
 * The equations of motion are generated through robcogen, using the following set of generalized coordinates:
 * (ballPosition_x, ballPosition_y, eulerAnglesZyx theta_z, eulerAnglesZyx theta_y, eulerAnglesZyx theta_x)
 * The control input are u = (torque_wheel1, torque_wheel2, torque_wheel3)
 */
class BallbotSystemDynamics : public SystemDynamicsBaseAD {
 public:
  /** Constructor */
  BallbotSystemDynamics(const std::string& libraryFolder, bool recompileLibraries) : SystemDynamicsBaseAD() {
    wheelRadius_ = param_.wheelRadius_;
    ballRadius_ = param_.ballRadius_;

    initialize(STATE_DIM, INPUT_DIM, "ballbot_dynamics", libraryFolder, recompileLibraries, true);
  }

  /** Destructor */
  ~BallbotSystemDynamics() override = default;

  BallbotSystemDynamics(const BallbotSystemDynamics& rhs) = default;

  BallbotSystemDynamics* clone() const override { return new BallbotSystemDynamics(*this); }

  ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                            const ad_vector_t& parameters) const override;

 private:
  BallbotParameters param_;
  scalar_t wheelRadius_;
  scalar_t ballRadius_;
};

}  // namespace ballbot
}  // namespace ocs2
