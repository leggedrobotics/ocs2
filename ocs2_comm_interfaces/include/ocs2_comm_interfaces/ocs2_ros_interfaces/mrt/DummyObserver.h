//
// Created by rgrandia on 10.02.20.
//

#pragma once

#include <ocs2_oc/oc_data/PrimalSolution.h>
#include "ocs2_comm_interfaces/CommandData.h"
#include "ocs2_comm_interfaces/SystemObservation.h"

namespace ocs2 {

/**
 * This class can be used to observe the dummy loop. Every loop of the dummy, the update method is called on all subscribed observers.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class DummyObserver {
 public:
  using primal_solution_t = PrimalSolution<STATE_DIM, INPUT_DIM>;
  using command_data_t = CommandData<STATE_DIM, INPUT_DIM>;
  using system_observation_t = SystemObservation<STATE_DIM, INPUT_DIM>;

  virtual ~DummyObserver() = default;

  /**
   * Update is called at the end of every timestep in the dummy loop.
   *
   * @param observation : system state and input at the current time
   * @param primalSolution : latest MPC primal solution
   * @param command : latest command on which the MPC solution is based
   */
  virtual void update(const system_observation_t& observation, const primal_solution_t& primalSolution, const command_data_t& command) = 0;
};
}  // namespace ocs2
