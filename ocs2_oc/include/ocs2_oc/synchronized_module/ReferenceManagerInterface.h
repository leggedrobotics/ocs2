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
#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_core/reference/TargetTrajectories.h>

namespace ocs2 {

/**
 * Defines the interface for providing the ModeSchedule and the TargetTrajectories to the solver.
 */
class ReferenceManagerInterface {
 public:
  /** Constructor */
  ReferenceManagerInterface() = default;

  /** Destructor */
  virtual ~ReferenceManagerInterface() = default;

  /** Disable copy / move */
  ReferenceManagerInterface& operator=(const ReferenceManagerInterface&) = delete;
  ReferenceManagerInterface(const ReferenceManagerInterface&) = delete;
  ReferenceManagerInterface& operator=(ReferenceManagerInterface&&) = delete;
  ReferenceManagerInterface(ReferenceManagerInterface&&) = delete;

  /**
   * The method is called right before the solver runs and before any other SolverSynchronizedModule::preSolverRun().
   *
   * @param [in] initTime : Start time of the optimization horizon.
   * @param [in] finalTime : Final time of the optimization horizon.
   * @param [in] initState : State at the start of the optimization horizon.
   */
  virtual void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState){};

  /** Returns a const reference to the active ModeSchedule. */
  virtual const ModeSchedule& getModeSchedule() const = 0;

  /**
   * Sets the ModeSchedule to the buffer. The buffer will move to active ModeSchedule once preSolverRun() is called.
   * @note: This method must be thread safe.
   */
  virtual void setModeSchedule(const ModeSchedule& modeSchedule) = 0;

  /**
   * Move the ModeSchedule to the buffer. The buffer will move to active ModeSchedule once preSolverRun() is called.
   * @note: This method must be thread safe.
   */
  virtual void setModeSchedule(ModeSchedule&& modeSchedule) = 0;

  /** Returns a const reference to the active TargetTrajectories. */
  virtual const TargetTrajectories& getTargetTrajectories() const = 0;

  /**
   * Sets the TargetTrajectories to the buffer. The buffer will move to active ModeSchedule once preSolverRun() is called.
   * @note: This method must be thread safe.
   */
  virtual void setTargetTrajectories(const TargetTrajectories& targetTrajectories) = 0;

  /**
   * Move the TargetTrajectories to the buffer. The buffer will move to active ModeSchedule once preSolverRun() is called.
   * @note: This method must be thread safe.
   */
  virtual void setTargetTrajectories(TargetTrajectories&& targetTrajectories) = 0;
};

}  // namespace ocs2
