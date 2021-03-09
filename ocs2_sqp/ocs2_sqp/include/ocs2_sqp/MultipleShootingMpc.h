//
// Created by rgrandia on 09.11.20.
//

#pragma once

#include <ocs2_mpc/MPC_BASE.h>

#include <ocs2_sqp/MultipleShootingSolver.h>

namespace ocs2 {
class MultipleShootingMpc : public MPC_BASE {
 public:
  MultipleShootingMpc(mpc::Settings mpcSettings, MultipleShootingSolverSettings settings, const SystemDynamicsBase* systemDynamicsPtr,
                      const CostFunctionBase* costFunctionPtr, const ConstraintBase* constraintPtr = nullptr,
                      const CostFunctionBase* terminalCostPtr = nullptr,
                      const SystemOperatingTrajectoriesBase* operatingTrajectoriesPtr = nullptr)
      : MPC_BASE(std::move(mpcSettings)) {
    solverPtr_.reset(new MultipleShootingSolver(std::move(settings), systemDynamicsPtr, costFunctionPtr, constraintPtr, terminalCostPtr,
                                                operatingTrajectoriesPtr));
  };

  ~MultipleShootingMpc() override = default;

  MultipleShootingSolver* getSolverPtr() override { return solverPtr_.get(); }
  const MultipleShootingSolver* getSolverPtr() const override { return solverPtr_.get(); }

 protected:
  void calculateController(scalar_t initTime, const vector_t& initState, scalar_t finalTime) override {
    // Hopefully this is enough
    scalar_array_t partitioningTimes = {0.0};
    solverPtr_->run(initTime, initState, finalTime, partitioningTimes);
  }

 private:
  std::unique_ptr<MultipleShootingSolver> solverPtr_;
};
}  // namespace ocs2