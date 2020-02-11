//
// Created by rgrandia on 10.02.20.
//

#pragma once

#include <ocs2_oc/oc_data/PrimalSolution.h>
#include "ocs2_comm_interfaces/CommandData.h"
#include "ocs2_comm_interfaces/SystemObservation.h"


namespace ocs2 {

  template <size_t STATE_DIM, size_t INPUT_DIM>
  class DummyObserver {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using primal_solution_t = PrimalSolution<STATE_DIM, INPUT_DIM>;
    using command_data_t = CommandData<STATE_DIM, INPUT_DIM>;
    using system_observation_t = SystemObservation<STATE_DIM, INPUT_DIM>;

    virtual void update(const system_observation_t& observation, const primal_solution_t& primalSolution, const command_data_t& command) = 0;


  };
}
