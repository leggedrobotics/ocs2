//
// Created by rgrandia on 18.03.20.
//

#pragma once

#include <ocs2_core/loopshaping/LoopshapingDefinition.h>
#include <ocs2_oc/oc_solver/SolverSynchronizedModule.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>

#include "ocs2_quadruped_loopshaping_interface/LoopshapingDimensions.h"

namespace switched_model_loopshaping {

class LoopshapingSynchronizedModule final : public ocs2::SolverSynchronizedModule<STATE_DIM, INPUT_DIM> {
 public:
  using switched_model_solver_module_t = ocs2::SolverSynchronizedModule<switched_model::STATE_DIM, switched_model::INPUT_DIM>;

  /** Vector of switched model solver synchronized modules that will be wrapped with loopshaping*/
  std::vector<std::shared_ptr<switched_model_solver_module_t>> synchronizedModules_;

  LoopshapingSynchronizedModule(std::vector<std::shared_ptr<switched_model_solver_module_t>> synchronizedModules,
                                std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition);

  ~LoopshapingSynchronizedModule() override = default;

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                    const ocs2::CostDesiredTrajectories& costDesiredTrajectory) override;

  void postSolverRun(const primal_solution_t& primalSolution) override;

 private:
  std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition_;
};

}  // namespace switched_model_loopshaping
