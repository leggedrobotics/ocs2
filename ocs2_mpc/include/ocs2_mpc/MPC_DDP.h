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
#include <ocs2_ddp/GaussNewtonDDP.h>

#include "ocs2_mpc/MPC_BASE.h"

namespace ocs2 {

/**
 * This is an MPC implementation with DDP (SLQ or ILQR) optimal control solvers.
 */
class MPC_DDP : public MPC_BASE {
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
  MPC_DDP(mpc::Settings mpcSettings, ddp::Settings ddpSettings, const RolloutBase& rollout,
          const OptimalControlProblem& optimalControlProblem, const Initializer& initializer);

  /** Default destructor. */
  ~MPC_DDP() override = default;

  GaussNewtonDDP* getSolverPtr() override { return ddpPtr_.get(); }

  const GaussNewtonDDP* getSolverPtr() const override { return ddpPtr_.get(); }

 protected:
  void calculateController(scalar_t initTime, const vector_t& initState, scalar_t finalTime) override;

 private:
  std::unique_ptr<GaussNewtonDDP> ddpPtr_;
};

}  // namespace ocs2
