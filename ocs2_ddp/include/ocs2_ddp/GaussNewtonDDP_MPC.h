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
#include <ocs2_ddp/ILQR.h>
#include <ocs2_ddp/SLQ.h>

#include "ocs2_mpc/MPC_BASE.h"

namespace ocs2 {

/**
 * This is an MPC implementation with GaussNewton DDP (SLQ or ILQR) optimal control solvers.
 */
class GaussNewtonDDP_MPC final : public MPC_BASE {
 public:
  /**
   * Constructor
   *
   * @param [in] mpcSettings: Structure containing the settings for the MPC algorithm.
   * @param [in] ddpSettings: Structure containing the settings for the DDP algorithm.
   * @param [in] rollout: The rollout class used for simulating the system dynamics.
   * @param [in] optimalControlProblem: The optimal control problem definition.
   * @param [in] initializer: This class initializes the state-input for the time steps that no controller is available.
   */
  GaussNewtonDDP_MPC(mpc::Settings mpcSettings, ddp::Settings ddpSettings, const RolloutBase& rollout,
                     const OptimalControlProblem& optimalControlProblem, const Initializer& initializer)
      : MPC_BASE(std::move(mpcSettings)) {
    switch (ddpSettings.algorithm_) {
      case ddp::Algorithm::SLQ:
        ddpPtr_.reset(new SLQ(std::move(ddpSettings), rollout, optimalControlProblem, initializer));
        break;
      case ddp::Algorithm::ILQR:
        ddpPtr_.reset(new ILQR(std::move(ddpSettings), rollout, optimalControlProblem, initializer));
        break;
      default:
        throw std::runtime_error("Undefined ddp::Algorithm type!");
    }
  }

  /** Default destructor. */
  ~GaussNewtonDDP_MPC() override = default;

  GaussNewtonDDP* getSolverPtr() override { return ddpPtr_.get(); }
  const GaussNewtonDDP* getSolverPtr() const override { return ddpPtr_.get(); }

 private:
  void calculateController(scalar_t initTime, const vector_t& initState, scalar_t finalTime) override {
    if (settings().coldStart_) {
      ddpPtr_->reset();
    }
    ddpPtr_->run(initTime, initState, finalTime);
  }

  std::unique_ptr<GaussNewtonDDP> ddpPtr_;
};

}  // namespace ocs2
