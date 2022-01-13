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
#include <ocs2_core/misc/Benchmark.h>

#include <ocs2_oc/oc_solver/SolverBase.h>

#include "ocs2_mpc/MPC_Settings.h"

namespace ocs2 {

/**
 * This class is an interface class for the MPC method.
 */
class MPC_BASE {
 public:
  /**
   * Constructor
   *
   * @param [in] mpcSettings: Structure containing the settings for the MPC algorithm.
   */
  explicit MPC_BASE(mpc::Settings mpcSettings);

  /** Destructor. */
  virtual ~MPC_BASE() = default;

  /**
   * Resets the class to its state after construction.
   * @note reset() must not be called while the solver is running.
   */
  virtual void reset();

  /**
   * The main routine of MPC which runs MPC for the given state and time.
   *
   * @param [in] currentTime: The given time.
   * @param [in] currentState: The given state.
   */
  virtual bool run(scalar_t currentTime, const vector_t& currentState);

  /** Gets a pointer to the underlying solver used in the MPC. */
  virtual SolverBase* getSolverPtr() = 0;

  /** Gets a const pointer to the underlying solver used in the MPC. */
  virtual const SolverBase* getSolverPtr() const = 0;

  /** Returns the time horizon for which the optimizer is called. */
  scalar_t getTimeHorizon() const { return mpcSettings_.timeHorizon_; }

  /** Gets the MPC settings. */
  const mpc::Settings& settings() const { return mpcSettings_; }

 protected:
  /**
   * Solves the optimal control problem for the given state and time period ([initTime,finalTime]).
   *
   * @param [out] initTime: Initial time. This value can be adjusted by the optimizer.
   * @param [in] initState: Initial state.
   * @param [in] finalTime: Final time. This value can be adjusted by the optimizer.
   */
  virtual void calculateController(scalar_t initTime, const vector_t& initState, scalar_t finalTime) = 0;

  /** Whether this is the first iteration of MPC or not. */
  bool isFirstMpcRun() const { return initRun_; }

 private:
  bool initRun_ = true;
  const mpc::Settings mpcSettings_;

  benchmark::RepeatedTimer mpcTimer_;
};

}  // namespace ocs2
