//
// Created by rgrandia on 09.11.20.
//

#include "ocs2_sqp/MultipleShootingSolver.h"

#include <ocs2_core/control/FeedforwardController.h>

namespace ocs2 {

MultipleShootingSolver::MultipleShootingSolver(MultipleShootingSolverSettings settings, const SystemDynamicsBase* systemDynamicsPtr, const CostFunctionBase* costFunctionPtr) : Solver_BASE(),
        settings_(std::move(settings)),
                                                                                                                                                                                systemDynamicsPtr_(systemDynamicsPtr->clone()),
                                                                                                                                                                                costFunctionPtr_(costFunctionPtr->clone())
{

}

void MultipleShootingSolver::reset() {
  Solver_BASE::reset();
  // additional reset
}

void MultipleShootingSolver::runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes) {
  // ignore partitioningTimes

  // Initialize cost
  costFunctionPtr_->setCostDesiredTrajectory(&this->getCostDesiredTrajectories());

  // Solve the problem.

  // Fill PrimalSolution. time, state , input
  primalSolution_.modeSchedule_ = this->getModeSchedule();
  primalSolution_.controllerPtr_.reset(new FeedforwardController(primalSolution_.timeTrajectory, primalSolution_.inputTrajectory);
}

}