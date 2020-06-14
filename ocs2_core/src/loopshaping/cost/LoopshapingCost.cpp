/******************************************************************************
Copyright (c) 2020, Ruben Grandia. All rights reserved.

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

#include <ocs2_core/loopshaping/cost/LoopshapingCost.h>
#include <ocs2_core/loopshaping/cost/LoopshapingCostEliminatePattern.h>
#include <ocs2_core/loopshaping/cost/LoopshapingCostInputPattern.h>
#include <ocs2_core/loopshaping/cost/LoopshapingCostOutputPattern.h>

namespace ocs2 {

std::unique_ptr<LoopshapingCost> LoopshapingCost::create(const CostFunctionBase& systemCost,
                                                         std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) {
  switch (loopshapingDefinition->getType()) {
    case LoopshapingType::outputpattern:
      return std::unique_ptr<LoopshapingCost>(new LoopshapingCostOutputPattern(systemCost, std::move(loopshapingDefinition)));
    case LoopshapingType::inputpattern:
      return std::unique_ptr<LoopshapingCost>(new LoopshapingCostInputPattern(systemCost, std::move(loopshapingDefinition)));
    case LoopshapingType::eliminatepattern:
      return std::unique_ptr<LoopshapingCost>(new LoopshapingCostEliminatePattern(systemCost, std::move(loopshapingDefinition)));
    default:
      throw std::runtime_error("[LoopshapingCost::create] invalid loopshaping type");
  }
}

void LoopshapingCost::setCurrentStateAndControl(scalar_t t, const vector_t& x, const vector_t& u) {
  costApproximationValid_ = false;
  costEvaluationValid_ = false;

  t_ = t;
  x_system_ = loopshapingDefinition_->getSystemState(x);
  u_system_ = loopshapingDefinition_->getSystemInput(x, u);
  x_filter_ = loopshapingDefinition_->getFilterState(x);
  u_filter_ = loopshapingDefinition_->getFilteredInput(x, u);

  CostFunctionBase::setCurrentStateAndControl(t, x, u);
}

void LoopshapingCost::setCostDesiredTrajectoriesPtr(const CostDesiredTrajectories* costDesiredTrajectoriesPtr) {
  if (costDesiredTrajectoriesPtr) {
    CostFunctionBase::setCostDesiredTrajectoriesPtr(costDesiredTrajectoriesPtr);

    // Desired trajectories are dynamic size -> must resize for future cast to fixed size vectors
    size_t reference_length = costDesiredTrajectoriesPtr->desiredTimeTrajectory().size();

    systemCostDesiredTrajectories_ = CostDesiredTrajectories(reference_length);
    systemCostDesiredTrajectories_.desiredTimeTrajectory() = costDesiredTrajectoriesPtr->desiredTimeTrajectory();
    auto& systemStateTrajectory = systemCostDesiredTrajectories_.desiredStateTrajectory();
    auto& systemInputTrajectory = systemCostDesiredTrajectories_.desiredInputTrajectory();
    const auto& stateTrajectory = costDesiredTrajectoriesPtr->desiredStateTrajectory();
    const auto& inputTrajectory = costDesiredTrajectoriesPtr->desiredInputTrajectory();

    assert(reference_length > 0);
    const size_t FILTER_STATE_DIM = loopshapingDefinition_->getInputFilter().getNumStates();
    const size_t FILTER_INPUT_DIM = loopshapingDefinition_->getInputFilter().getNumInputs();
    const size_t SYSTEM_STATE_DIM = stateTrajectory.front().rows() - FILTER_STATE_DIM;
    const size_t SYSTEM_INPUT_DIM = inputTrajectory.front().rows() - FILTER_INPUT_DIM;

    for (int k = 0; k < reference_length; k++) {
      // For now assume that cost DesiredTrajectory is specified w.r.t original system x, u
      systemStateTrajectory[k] = stateTrajectory[k].head(SYSTEM_STATE_DIM);
      systemInputTrajectory[k] = inputTrajectory[k].head(SYSTEM_INPUT_DIM);
    }

    systemCost_->setCostDesiredTrajectoriesPtr(&systemCostDesiredTrajectories_);
  }
}

scalar_t LoopshapingCost::getCost() {
  computeCost();
  const auto& gamma = loopshapingDefinition_->gamma_;
  return gamma * c_filter_ + (1.0 - gamma) * c_system_;
};

scalar_t LoopshapingCost::getCostDerivativeTime() {
  computeApproximation();
  // TODO(rgrandia)
  return 0;
}

scalar_t LoopshapingCost::getTerminalCost() {
  systemCost_->setCurrentStateAndControl(t_, x_system_, u_system_);
  return systemCost_->getTerminalCost();
};

scalar_t LoopshapingCost::getTerminalCostDerivativeTime() {
  return 0;
}

vector_t LoopshapingCost::getTerminalCostDerivativeState() {
  systemCost_->setCurrentStateAndControl(t_, x_system_, u_system_);
  vector_t dPhidx_system = systemCost_->getTerminalCostDerivativeState();
  const size_t FILTER_STATE_DIM = loopshapingDefinition_->getInputFilter().getNumStates();
  const size_t SYSTEM_STATE_DIM = dPhidx_system.rows();
  vector_t dPhidx(SYSTEM_STATE_DIM + FILTER_STATE_DIM);
  dPhidx.head(SYSTEM_STATE_DIM) = dPhidx_system;
  dPhidx.tail(FILTER_STATE_DIM).setZero();
  return dPhidx;
};

matrix_t LoopshapingCost::getTerminalCostSecondDerivativeState() {
  systemCost_->setCurrentStateAndControl(t_, x_system_, u_system_);
  matrix_t dPhidxx_system = systemCost_->getTerminalCostSecondDerivativeState();
  const size_t FILTER_STATE_DIM = loopshapingDefinition_->getInputFilter().getNumStates();
  const size_t SYSTEM_STATE_DIM = dPhidxx_system.rows();
  matrix_t dPhidxx = matrix_t::Zero(SYSTEM_STATE_DIM + FILTER_STATE_DIM, SYSTEM_STATE_DIM + FILTER_STATE_DIM);
  dPhidxx.block(0, 0, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM) = dPhidxx_system;
  return dPhidxx;
};

void LoopshapingCost::computeCost() {
  if (!costEvaluationValid_) {
    systemCost_->setCurrentStateAndControl(t_, x_system_, u_system_);
    c_system_ = systemCost_->getCost();

    systemCost_->setCurrentStateAndControl(t_, x_system_, u_filter_);
    c_filter_ = systemCost_->getCost();

    costEvaluationValid_ = true;
  }
};

void LoopshapingCost::computeApproximation() {
  if (!costApproximationValid_) {
    systemCost_->setCurrentStateAndControl(t_, x_system_, u_system_);
    Q_system_ = systemCost_->getCostSecondDerivativeState();
    P_system_ = systemCost_->getCostDerivativeInputState();
    R_system_ = systemCost_->getCostSecondDerivativeInput();
    q_system_ = systemCost_->getCostDerivativeState();
    r_system_ = systemCost_->getCostDerivativeInput();
    c_system_ = systemCost_->getCost();

    systemCost_->setCurrentStateAndControl(t_, x_system_, u_filter_);
    Q_filter_ = systemCost_->getCostSecondDerivativeState();
    P_filter_ = systemCost_->getCostDerivativeInputState();
    R_filter_ = systemCost_->getCostSecondDerivativeInput();
    q_filter_ = systemCost_->getCostDerivativeState();
    r_filter_ = systemCost_->getCostDerivativeInput();
    c_filter_ = systemCost_->getCost();

    costEvaluationValid_ = true;
    costApproximationValid_ = true;
  }
};

}  // namespace ocs2
