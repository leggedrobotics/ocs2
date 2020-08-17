//
// Created by rgrandia on 18.03.20.
//

#pragma once

#include <ocs2_core/loopshaping/LoopshapingDefinition.h>
#include <ocs2_oc/oc_solver/SolverSynchronizedModule.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>

#include "ocs2_quadruped_loopshaping_interface/LoopshapingDimensions.h"

namespace switched_model_loopshaping {

class LoopshapingSynchronizedModule final : public ocs2::SolverSynchronizedModule {
 public:
  /** Vector of switched model solver synchronized modules that will be wrapped with loopshaping*/
  std::vector<std::shared_ptr<ocs2::SolverSynchronizedModule>> synchronizedModules_;

  LoopshapingSynchronizedModule(std::vector<std::shared_ptr<ocs2::SolverSynchronizedModule>> synchronizedModules,
                                std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition);

  ~LoopshapingSynchronizedModule() override = default;

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ocs2::CostDesiredTrajectories& costDesiredTrajectory) override;

  void postSolverRun(const ocs2::PrimalSolution& primalSolution) override;

 private:
  std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition_;
};

}  // namespace switched_model_loopshaping
