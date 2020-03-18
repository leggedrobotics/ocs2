//
// Created by rgrandia on 18.03.20.
//

#pragma once

#include "ocs2_oc/oc_solver/SolverSynchronizedModule.h"

#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

class GaitReceiver : public ocs2::SolverSynchronizedModule<STATE_DIM, INPUT_DIM> {
 public:

  GaitReceiver

  ~GaitReceiver() override = default;

  virtual void preSolverRun(scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                            const ocs2::CostDesiredTrajectories& costDesiredTrajectory);

  virtual void postSolverRun(const primal_solution_t& primalSolution) {};
};
};

}  // namespace switched_model
