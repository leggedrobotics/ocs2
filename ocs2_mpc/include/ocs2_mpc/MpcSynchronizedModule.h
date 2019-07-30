//
// Created by rgrandia on 27.07.19.
//

#ifndef OCS2_CTRL_MPCSYNCHRONIZEDMODULE_H
#define OCS2_CTRL_MPCSYNCHRONIZEDMODULE_H

#include <ocs2_core/cost/CostDesiredTrajectories.h>
#include <ocs2_core/logic/rules/HybridLogicRules.h>

namespace ocs2 {
/**
 * An MPC synchronized module is updated once before an mpc problem is solved
 */
template<typename scalar_t = double>
class MpcSynchronizedModule {
 public:
  virtual ~MpcSynchronizedModule() = default;
  virtual void update(scalar_t initTime, scalar_t finalTime, const Eigen::Matrix<scalar_t, -1, 1>& currentState, const CostDesiredTrajectories<scalar_t>& costDesiredTrajectory, const HybridLogicRules* hybridLogicRules) = 0;
};
} // namespace ocs2

#endif //OCS2_CTRL_MPCSYNCHRONIZEDMODULE_H
