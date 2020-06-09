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
#include <ocs2_core/control/ControllerBase.h>
#include <ocs2_core/cost/CostDesiredTrajectories.h>
#include <ocs2_core/misc/Benchmark.h>

#include <ocs2_oc/oc_solver/Solver_BASE.h>

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
   * @param [in] partitioningTimes: Array of times to divide up the horizon
   * @param [in] mpcSettings: Structure containing the settings for the MPC algorithm.
   */
  MPC_BASE(const scalar_array_t& partitioningTimes, MPC_Settings mpcSettings);

  /**
   * destructor.
   */
  virtual ~MPC_BASE() = default;

  /**
   * Resets the class to its state after construction.
   */
  virtual void reset();

  /**
   * The main routine of MPC which runs MPC for the given state and time.
   *
   * @param [in] currentTime: The given time.
   * @param [in] currentState: The given state.
   */
  virtual bool run(const scalar_t& currentTime, const vector_t& currentState);

  /**
   * Solves the optimal control problem for the given state and time period ([initTime,finalTime]).
   *
   * @param [out] initTime: Initial time. This value can be adjusted by the optimizer.
   * @param [in] initState: Initial state.
   * @param [in] finalTime: Final time. This value can be adjusted by the optimizer.
   */
  virtual void calculateController(const scalar_t& initTime, const vector_t& initState, const scalar_t& finalTime) = 0;

  /**
   * Gets a pointer to the underlying solver used in the MPC.
   *
   * @return A pointer to the underlying solver used in the MPC
   */
  virtual Solver_BASE* getSolverPtr() = 0;

  /**
   * Gets a const pointer to the underlying solver used in the MPC.
   *
   * @return A const pointer to the underlying solver used in the MPC
   */
  virtual const Solver_BASE* getSolverPtr() const = 0;

  /**
   * Returns the initial time for which the optimizer is called.
   *
   * @return Initial time
   */
  virtual scalar_t getStartTime() const;

  /**
   * Returns the final time for which the optimizer is called.
   *
   * @return Final time
   */
  virtual scalar_t getFinalTime() const;

  /**
   * Returns the time horizon for which the optimizer is called.
   *
   * @return Time horizon
   */
  virtual scalar_t getTimeHorizon() const;

  /**
   * Gets partitioning time.
   *
   * @param [out] Partitioning times
   */
  virtual void getPartitioningTimes(scalar_array_t& partitioningTimes) const;

  /**
   * Gets the MPC settings.
   *
   * @return structure which details MPC settings
   */
  const MPC_Settings& settings() const;

 protected:
  /**
   * Rewinds MPC.
   */
  virtual void rewind();

  /**
   * Adjustments time horizon.
   *
   * @param [in] partitioningTimes: Partitioning times after rewind.
   * @param [out] initTime: Adjustments initial time.
   * @param [out] finalTime: Adjustments final time.
   * @param [out] initActivePartitionIndex: Index of the initial active partition.
   * @param [out] finalActivePartitionIndex: Index of the final active partition.
   */
  virtual void adjustmentTimeHorizon(const scalar_array_t& partitioningTimes, scalar_t& initTime, scalar_t& finalTime,
                                     size_t& initActivePartitionIndex, size_t& finalActivePartitionIndex) const;

  /*************
   * Variables *
   *************/
  MPC_Settings mpcSettings_;

  bool initRun_ = true;

  benchmark::RepeatedTimer mpcTimer_;

  size_t initnumPartitions_;
  scalar_array_t initPartitioningTimes_;
  size_t numPartitions_ = 0;
  scalar_array_t partitioningTimes_{};

  size_t initActivePartitionIndex_ = 0;
  size_t finalActivePartitionIndex_ = 0;

  scalar_t lastControlDesignTime_;
};

}  // namespace ocs2
