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

#include "ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h"

#include "DummyObserver.h"

namespace ocs2 {

/**
 * This class implements a loop to test MPC-MRT communication interface using ROS.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class MRT_ROS_Dummy_Loop {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using mrt_t = MRT_ROS_Interface<STATE_DIM, INPUT_DIM>;
  using system_observation_t = typename mrt_t::system_observation_t;
  using primal_solution_t = typename mrt_t::primal_solution_t;
  using command_data_t = typename mrt_t::command_data_t;
  using scalar_t = typename mrt_t::scalar_t;
  using state_vector_t = typename mrt_t::state_vector_t;

  using observer_t = DummyObserver<STATE_DIM, INPUT_DIM>;

  /**
   * Constructor.
   *
   * @param [in] mrt: The underlying MRT class to be used. If MRT contains a rollout object, the dummy will roll out
   * the received controller using the MRT::rolloutPolicy() method instead of just sending back a planned state.
   * @param [in] mrtDesiredFrequency: MRT loop frequency in Hz. This should always set to a positive number.
   * @param [in] mpcDesiredFrequency: MPC loop frequency in Hz. If set to a positive number, MPC loop
   * will be simulated to run by this frequency. Note that this might not be the MPC's real-time frequency.
   */
  MRT_ROS_Dummy_Loop(mrt_t& mrt, scalar_t mrtDesiredFrequency, scalar_t mpcDesiredFrequency = -1);

  /**
   * Destructor.
   */
  virtual ~MRT_ROS_Dummy_Loop() = default;

  /**
   * Runs the dummy MRT loop.
   *
   * @param [in] initObservation: The initial observation.
   * @param [in] initCostDesiredTrajectories: The initial desired cost trajectories.
   */
  void run(const system_observation_t& initObservation, const CostDesiredTrajectories& initCostDesiredTrajectories);

  /**
   * Subscribe a set of observers to the dummy loop. Observers are updated in the provided order at the end of each timestep.
   * The previous list of observers is overwritten.
   *
   * @param observers : vector of observers.
   */
  void subscribeObservers(const std::vector<std::shared_ptr<observer_t>>& observers) { observers_ = observers; }

 protected:
  /**
   * A user-defined function which modifies the observation before publishing.
   *
   * @param [in] observation: The current observation.
   */
  virtual void modifyObservation(system_observation_t& observation) {}

 private:
  mrt_t& mrt_;
  scalar_t mrtDesiredFrequency_;
  scalar_t mpcDesiredFrequency_;

  bool realtimeLoop_;

  system_observation_t observation_;
  std::vector<std::shared_ptr<observer_t>> observers_;
};

}  // namespace ocs2

#include "implementation/MRT_ROS_Dummy_Loop.h"
