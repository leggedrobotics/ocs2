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

#include <condition_variable>
#include <csignal>
#include <ctime>
#include <iostream>
#include <string>
#include <thread>

#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_core/model_data/Multiplier.h>
#include "ocs2_mpc/MPC_BASE.h"
#include "ocs2_mpc/MRT_BASE.h"

namespace ocs2 {

/**
 * A lean ROS independent interface to OCS2. In incorporates the functionality of the MPC and the MRT (trajectory tracking) modules.
 * Please refer to ocs2_double_integrator_example for a minimal example and tests
 */
class MPC_MRT_Interface final : public MRT_BASE {
 public:
  /**
   * Constructor
   * @param [in] mpc: The underlying MPC class to be used.
   */
  explicit MPC_MRT_Interface(MPC_BASE& mpc);

  ~MPC_MRT_Interface() override = default;

  void resetMpcNode(const TargetTrajectories& initTargetTrajectories) override;

  void setCurrentObservation(const SystemObservation& currentObservation) override;

  /*
   * Gets the ReferenceManager which manages both ModeSchedule and TargetTrajectories.
   */
  ReferenceManagerInterface& getReferenceManager();
  const ReferenceManagerInterface& getReferenceManager() const;

  /**
   * Advance the mpc module for one iteration. The evaluation methods can be called while this method is running. They will evaluate the
   * control law that was up-to-date at the last updatePolicy() call.
   */
  void advanceMpc();

  /**
   * @brief Retrieves the gain matrix from solver capable of optimizing over LinearController type.
   *
   * @note This method is not thread-safe, meaning you can only access this data safely at the end of each MPC iteration.
   *
   * @param [in] time: query time
   * @return The gain matrix.
   */
  matrix_t getLinearFeedbackGain(scalar_t time);

  /**
   * @brief Access the solver's internal value function.
   *
   * @note This method is not thread-safe, meaning you can only access this data safely at the end of each MPC iteration.
   *
   * @param [in] time: query time
   * @param [in] state: query state
   * @return The quadratic approximation of the value function at the requested time and state.
   */
  ScalarFunctionQuadraticApproximation getValueFunction(scalar_t time, const vector_t& state) const;

  /**
   * @brief Computes the Lagrange multiplier related to the state-input constraints
   *
   * @note This method is not thread-safe, meaning you can only access this data safely at the end of each MPC iteration.
   *
   * @param [in] time: query time
   * @param [in] state: query state
   * @return The Lagrange multiplier at the requested time and state
   */
  vector_t getStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t& state) const;

  /**
   * @brief Returns the intermediate dual solution at the given time.
   *
   * @note This method is not thread-safe, meaning you can only access this data safely at the end of each MPC iteration.
   *
   * @param [in] time: The inquiry time
   * @return The collection of multipliers associated to state/state-input, equality/inequality Lagrangian terms.
   */
  MultiplierCollection getIntermediateDualSolution(scalar_t time) const;

 private:
  /**
   * Updates the buffer variables from the MPC object. This method is automatically called by advanceMpc()
   *
   * @param [in] mpcInitObservation: The observation used to run the MPC.
   */
  void copyToBuffer(const SystemObservation& mpcInitObservation);

  MPC_BASE& mpc_;
  benchmark::RepeatedTimer mpcTimer_;

  // MPC inputs
  SystemObservation currentObservation_;
  std::mutex observationMutex_;
};

}  // namespace ocs2
