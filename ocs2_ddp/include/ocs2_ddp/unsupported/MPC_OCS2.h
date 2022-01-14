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

#include <condition_variable>
#include <mutex>
#include <thread>

#include <ocs2_ddp/DDP_DataCollector.h>
#include <ocs2_ocs2/FrankWolfeGDDP.h>

#include "ocs2_ddp/GaussNewtonDDP_MPC.h"

namespace ocs2 {

/**
 * This an MPC implementation with OCS2 optimal control solver.
 */
class MPC_OCS2 : public GaussNewtonDDP_MPC {
 public:
  /**
   * Constructor
   *
   * @param [in] rolloutPtr: The rollout class used for simulating the system dynamics.
   * @param [in] systemDynamicsPtr: The system dynamics derivatives for subsystems of the system.
   * @param [in] systemConstraintsPtr: The system constraint function and its derivatives for subsystems.
   * @param [in] costFunctionPtr: The cost function (intermediate and terminal costs) and its derivatives for subsystems.
   * @param [in] operatingTrajectoriesPtr: The operating trajectories of system which will be used for initialization of SLQ.
   * @param [in] partitioningTimes: This will be used as the initial time partitioning. As the MPC progresses the internal
   * partitioningTimes will be shifted in time automatically.
   * @param [in] slqSettings: Structure containing the settings for the SLQ algorithm.
   * @param [in] gddpSettings: Structure containing the settings for the GDDP algorithm.
   * @param [in] mpcSettings: Structure containing the settings for the MPC algorithm.
   * @param [in] heuristicsFunctionPtr: Heuristic function used in the infinite time optimal control formulation. If it is not
   * defined, we will use the terminal cost function defined in costFunctionPtr.
   */
  MPC_OCS2(const RolloutBase* rolloutPtr, const SystemDynamicsBase* systemDynamicsPtr, const ConstraintBase* systemConstraintsPtr,
           const CostFunctionBase* costFunctionPtr, const SystemOperatingTrajectoriesBase* operatingTrajectoriesPtr,
           const scalar_array_t& partitioningTimes, const SLQ_Settings& slqSettings = SLQ_Settings(),
           const GDDP_Settings& gddpSettings = GDDP_Settings(), const MPC_Settings& mpcSettings = MPC_Settings(),
           const CostFunctionBase* heuristicsFunctionPtr = nullptr);

  /**
   * Destructor.
   */
  virtual ~MPC_OCS2();

  /**
   * Resets the class to its state after construction.
   */
  void reset() override;

  /**
   * The main routine of MPC which runs MPC for the given state and time.
   *
   * @param [in] currentTime: The given time.
   * @param [in] currentState: The given state.
   */
  virtual bool run(const scalar_t& currentTime, const vector_t& currentState);

 protected:
  /**
   * Rewinds the SLQ-MPC.
   */
  virtual void rewind();

  void runOCS2();

 private:
  std::unique_ptr<FrankWolfeGDDP> gddpPtr_;

  std::thread workerOCS2_;

  std::mutex dataCollectorMutex_;
  bool activateOCS2_;
  std::atomic_bool terminateOCS2_;
  std::condition_variable ocs2Synchronization_;

  std::unique_ptr<DDP_DataCollector> slqDataCollectorPtr_;

  scalar_array_t eventTimesOptimized_;
  size_array_t modeSequenceOptimized_;
};

}  // namespace ocs2
