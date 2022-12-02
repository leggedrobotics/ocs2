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
#include <ocs2_legged_robot_raisim/LeggedRobotRaisimConversions.h>
#include <ocs2_mpcnet_core/MpcnetInterfaceBase.h>

namespace ocs2 {
namespace legged_robot {

/**
 *  Legged robot MPC-Net interface between C++ and Python.
 */
class LeggedRobotMpcnetInterface final : public ocs2::mpcnet::MpcnetInterfaceBase {
 public:
  /**
   * Constructor.
   * @param [in] nDataGenerationThreads : Number of data generation threads.
   * @param [in] nPolicyEvaluationThreads : Number of policy evaluation threads.
   * @param [in] raisim : Whether to use RaiSim for the rollouts.
   */
  LeggedRobotMpcnetInterface(size_t nDataGenerationThreads, size_t nPolicyEvaluationThreads, bool raisim);

  /**
   * Default destructor.
   */
  ~LeggedRobotMpcnetInterface() override = default;

 private:
  /**
   * Helper to get the MPC.
   * @param [in] leggedRobotInterface : The legged robot interface.
   * @return Pointer to the MPC.
   */
  std::unique_ptr<MPC_BASE> getMpc(LeggedRobotInterface& leggedRobotInterface);

  // Legged robot interface pointers (keep alive for Pinocchio interface)
  std::vector<std::unique_ptr<LeggedRobotInterface>> leggedRobotInterfacePtrs_;
  // Legged robot RaiSim conversions pointers (keep alive for RaiSim rollout)
  std::vector<std::unique_ptr<LeggedRobotRaisimConversions>> leggedRobotRaisimConversionsPtrs_;
};

}  // namespace legged_robot
}  // namespace ocs2
