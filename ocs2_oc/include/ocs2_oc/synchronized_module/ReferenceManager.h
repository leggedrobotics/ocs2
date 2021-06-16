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

#include <atomic>
#include <mutex>

#include <ocs2_core/Types.h>
#include <ocs2_core/cost/CostDesiredTrajectories.h>
#include <ocs2_core/logic/ModeSchedule.h>

namespace ocs2 {

/**
 * Manages the ModeSchedule of the solver.
 */
class ReferenceManager {
 public:
  ReferenceManager() = default;
  virtual ~ReferenceManager() = default;
  ReferenceManager(ReferenceManager&& other) = default;
  ReferenceManager& operator=(ReferenceManager&& other) = default;

  ReferenceManager(const ReferenceManager& other) = delete;
  ReferenceManager& operator=(const ReferenceManager& other) = delete;

  /**
   * Method called right before the solver runs
   *
   * @param initTime : start time of the MPC horizon
   * @param finalTime : Final time of the MPC horizon
   * @param initState : State at the start of the MPC horizon
   */
  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState);

  /**
   * Returns a const reference to ModeSchedule. This method is NOT thread safe.
   */
  const ModeSchedule& getModeSchedule() const { return modeSchedule_; }

  /**
   * Returns a const reference to ModeSchedule. This method is NOT thread safe.
   */
  const CostDesiredTrajectories& getCostDesiredTrajectories() const { return costDesiredTrajectories_; }

  /**
   * Returns a a copy of ModeSchedule. This method is thread safe.
   */
  ModeSchedule getModeScheduleImage() const;

  /**
   * Returns a a copy of ModeSchedule. This method is thread safe.
   */
  CostDesiredTrajectories getCostDesiredTrajectoriesImage() const;

  /**
   * Sets the ModeSchedule to the buffer. The buffer will move to internal ModeSchedule once preSolverRun() is called.
   * This method is thread safe.
   */
  void setModeSchedule(const ModeSchedule& modeSchedule);

  /**
   * Sets the ModeSchedule to the buffer. The buffer will move to internal ModeSchedule once preSolverRun() is called.
   * This method is thread safe.
   */
  void setModeSchedule(ModeSchedule&& modeSchedule);

  /**
   * Sets the CostDesiredTrajectories to the buffer. The buffer will move to internal CostDesiredTrajectories once
   * preSolverRun() is called. This method is thread safe.
   */
  void setCostDesiredTrajectories(const CostDesiredTrajectories& costDesiredTrajectories);

  /**
   * Sets the CostDesiredTrajectories to the buffer. The buffer will move to internal CostDesiredTrajectories once
   * preSolverRun() is called. This method is thread safe.
   */
  void setCostDesiredTrajectories(CostDesiredTrajectories&& costDesiredTrajectories);

 protected:
  /**
   * This method modifies the active ModeSchedule and CostDesiredTrajectories.
   *
   * @param [in] initTime : start time of the MPC horizon.
   * @param [in] finalTime : Final time of the MPC horizon.
   * @param [in] initState : State at the start of the MPC horizon.
   * @param [in/out] modeSchedule : The active ModeSchedule.
   * @param [in/out] costDesiredTrajectory : The active CostDesiredTrajectories.
   */
  virtual void modifyActiveReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, ModeSchedule& modeSchedule,
                                      CostDesiredTrajectories& costDesiredTrajectory) {}

 private:
  ModeSchedule modeSchedule_;
  ModeSchedule modeScheduleBuffer_;

  CostDesiredTrajectories costDesiredTrajectories_;
  CostDesiredTrajectories costDesiredTrajectoriesBuffer_;

  mutable std::mutex dataMutex_;
  bool modeScheduleUpdated_{false};
  bool costDesiredTrajectoriesUpdated_{false};
};

}  // namespace ocs2
