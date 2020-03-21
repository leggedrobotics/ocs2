
#pragma once

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/cost/CostDesiredTrajectories.h>

#include "ocs2_oc/oc_data/PrimalSolution.h"

namespace ocs2 {

/**
 * A Solver synchronized module is updated once before and once after a problem is solved.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class SolverSynchronizedModule {
 public:
  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;

  using primal_solution_t = PrimalSolution<STATE_DIM, INPUT_DIM>;

  /**
   * Default destructor
   */
  virtual ~SolverSynchronizedModule() = default;

  /**
   * Method called right before the solver runs
   *
   * @param initTime : start time of the MPC horizon
   * @param finalTime : Final time of the MPC horizon
   * @param currentState : State at the start of the MPC horizon
   * @param costDesiredTrajectory : User defined cost desired trajectory
   */
  virtual void preSolverRun(scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                            const CostDesiredTrajectories& costDesiredTrajectory) = 0;

  /**
   * Method called right after the solver runs
   *
   * @param primalSolution : primalSolution
   */
  virtual void postSolverRun(const primal_solution_t& primalSolution) = 0;
};

}  // namespace ocs2
