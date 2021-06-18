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
#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_core/reference/TargetTrajectories.h>

namespace ocs2 {

/**
 * Manages the ModeSchedule and the TargetTrajectories of the solver.
 */
class ReferenceManager {
 public:
  /**
   * Constructor
   * @param [in] (optional) initialTargetTrajectories: Initial TargetTrajectories.
   * @param [in] (optional) initialModeSchedule: Initial ModeSchedule.
   */
  explicit ReferenceManager(TargetTrajectories initialTargetTrajectories = TargetTrajectories(),
                            ModeSchedule initialModeSchedule = ModeSchedule());

  virtual ~ReferenceManager() = default;
  ReferenceManager(ReferenceManager&& other) = default;
  ReferenceManager& operator=(ReferenceManager&& other) = default;

  ReferenceManager(const ReferenceManager& other) = delete;
  ReferenceManager& operator=(const ReferenceManager& other) = delete;

  /**
   * The method is called right before the solver runs and before any other SolverSynchronizedModule::preSolverRun().
   *
   * @param [in] initTime : Start time of the optimization horizon.
   * @param [in] finalTime : Final time of the optimization horizon.
   * @param [in] initState : State at the start of the optimization horizon.
   */
  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState);

  /**
   * Returns a const reference to ModeSchedule. This method is NOT thread safe.
   */
  const ModeSchedule& getModeSchedule() const { return modeSchedule_; }

  /**
   * Returns a const reference to ModeSchedule. This method is NOT thread safe.
   */
  const TargetTrajectories& getTargetTrajectories() const { return targetTrajectories_; }

  /**
   * Returns a a copy of ModeSchedule. This method is thread safe.
   */
  ModeSchedule getModeScheduleImage() const;

  /**
   * Returns a a copy of ModeSchedule. This method is thread safe.
   */
  TargetTrajectories getTargetTrajectoriesImage() const;

  /**
   * Sets the ModeSchedule to the buffer. The buffer will move to active ModeSchedule once preSolverRun() is called.
   * This method is thread safe.
   */
  void setModeSchedule(const ModeSchedule& modeSchedule);

  /**
   * Sets the ModeSchedule to the buffer. The buffer will move to active ModeSchedule once preSolverRun() is called.
   * This method is thread safe.
   */
  void setModeSchedule(ModeSchedule&& modeSchedule);

  /**
   * Sets the TargetTrajectories to the buffer. The buffer will move to active TargetTrajectories once
   * preSolverRun() is called. This method is thread safe.
   */
  void setTargetTrajectories(const TargetTrajectories& targetTrajectories);

  /**
   * Sets the TargetTrajectories to the buffer. The buffer will move to active TargetTrajectories once
   * preSolverRun() is called. This method is thread safe.
   */
  void setTargetTrajectories(TargetTrajectories&& targetTrajectories);

  /**
   * Swaps the ModeSchedule and TargetTrajectories to the active references. This method is NOT thread safe.
   */
  void swapReferences(TargetTrajectories& otherTargetTrajectories, ModeSchedule& otherModeSchedule);

 private:
  /**
   * Modifies the active ModeSchedule and TargetTrajectories.
   *
   * @param [in] initTime : Start time of the optimization horizon.
   * @param [in] finalTime : Final time of the optimization horizon.
   * @param [in] initState : State at the start of the optimization horizon.
   * @param [in, out] targetTrajectories : The updated TargetTrajectories. If setTargetTrajectories() has been called before,
   * TargetTrajectories is already updated by the set value.
   * @param [in, out] modeSchedule : The updated ModeSchedule. If setModeSchedule() has been called before, modeSchedule is
   * already updated by the set value.
   */
  virtual void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                                ModeSchedule& modeSchedule) {}

  TargetTrajectories targetTrajectories_;
  TargetTrajectories targetTrajectoriesBuffer_;

  ModeSchedule modeSchedule_;
  ModeSchedule modeScheduleBuffer_;

  mutable std::mutex dataMutex_;
  bool modeScheduleUpdated_{false};
  bool targetTrajectoriesUpdated_{false};
};

}  // namespace ocs2
