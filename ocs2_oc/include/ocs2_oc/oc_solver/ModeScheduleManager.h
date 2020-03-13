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

#include <ocs2_core/logic/ModeSchedule.h>

#include "ocs2_oc/oc_solver/SolverSynchronizedModule.h"

namespace ocs2 {

/**
 * Manages the ModeSchedule of the solver.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class ModeScheduleManager : public SolverSynchronizedModule<STATE_DIM, INPUT_DIM> {
 public:
  using Base = SolverSynchronizedModule<STATE_DIM, INPUT_DIM>;
  using typename Base::primal_solution_t;
  using typename Base::scalar_t;
  using typename Base::state_vector_t;

  //! default constructor
  explicit ModeScheduleManager(ModeSchedule modeSchedule) : modeSchedule_(std::move(modeSchedule)) {}

  //! Default destructor
  virtual ~ModeScheduleManager() = default;

  virtual void preSolverRun(scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                            const CostDesiredTrajectories& costDesiredTrajectory) {}

  virtual void postSolverRun(const primal_solution_t& primalSolution) {}

  /**
   * Returns a const reference to ModeSchedule.
   */
  virtual const ModeSchedule& modeSchedule() const { return modeSchedule_; }

  /**
   * Returns a reference to ModeSchedule.
   */
  virtual ModeSchedule& modeSchedule() { return modeSchedule_; }

 private:
  ModeSchedule modeSchedule_;
};

}  // namespace ocs2
