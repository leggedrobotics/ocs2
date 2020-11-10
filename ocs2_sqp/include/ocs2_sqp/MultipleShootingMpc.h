//
// Created by rgrandia on 09.11.20.
//

#pragma once

#include <ocs2_mpc/MPC_BASE.h>

#include <ocs2_sqp/MultipleShootingSolver.h>

namespace ocs2
{
  class MultipleShootingMpc : public MPC_BASE
  {
  public:
    MultipleShootingMpc(mpc::Settings mpcSettings,
                        MultipleShootingSolverSettings settings,
                        const ocs2::SystemDynamicsBaseAD *systemDynamicsPtr,
                        const ocs2::CostFunctionBase *costFunctionPtr)
        : MPC_BASE(std::move(mpcSettings))
    {
      solverPtr_.reset(new ocs2::MultipleShootingSolver(settings, systemDynamicsPtr, costFunctionPtr));
    };

    ~MultipleShootingMpc() override = default;

    Solver_BASE *getSolverPtr() override { solverPtr_.get(); }
    const Solver_BASE *getSolverPtr() const override { solverPtr_.get(); }

  protected:
    void calculateController(scalar_t initTime, const vector_t &initState, scalar_t finalTime)
    {
      // Hopefully this is enough
      scalar_array_t partitioningTimes = {0.0};
      solverPtr_->run(initTime, initState, finalTime, partitioningTimes);
    }

  private:
    std::unique_ptr<MultipleShootingSolver> solverPtr_;
  };
} // namespace ocs2