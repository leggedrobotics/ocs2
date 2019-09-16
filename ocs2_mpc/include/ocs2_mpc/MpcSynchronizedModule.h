
#pragma once

#include <ocs2_core/cost/CostDesiredTrajectories.h>
#include <ocs2_core/logic/rules/HybridLogicRules.h>

namespace ocs2 {

/**
 * An MPC synchronized module is updated once before an mpc problem is solved
 */
template <size_t STATE_DIM, typename scalar_t = double>
class MpcSynchronizedModule {
 public:
  //! Default destructor
  virtual ~MpcSynchronizedModule() = default;

  /**
   * Method called right before an MPC iteration
   *
   * @param initTime : start time of the MPC horizon
   * @param finalTime : Final time of the MPC horizon
   * @param currentState : State at the start of the MPC horizon
   * @param costDesiredTrajectory : User defined cost desired trajectory
   * @param hybridLogicRules : Logic rules containing discrete event time information
   */
  virtual void update(scalar_t initTime, scalar_t finalTime, const Eigen::Matrix<scalar_t, STATE_DIM, 1>& currentState,
                      const CostDesiredTrajectories<scalar_t>& costDesiredTrajectory, const HybridLogicRules* hybridLogicRules) = 0;
};

}  // namespace ocs2
