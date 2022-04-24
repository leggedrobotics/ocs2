#pragma once

#include "ocs2_pipg/mpc/PipgMpcSolver.h"

#include <ocs2_mpc/MPC_BASE.h>

namespace ocs2 {
class PipgMpc final : public MPC_BASE {
 public:
  /**
   * Constructor
   *
   * @param mpcSettings : settings for the mpc wrapping of the solver. Do not use this for maxIterations and stepsize, use multiple shooting
   * settings directly.
   * @param settings : settings for the multiple shooting solver.
   * @param [in] optimalControlProblem: The optimal control problem formulation.
   * @param [in] initializer: This class initializes the state-input for the time steps that no controller is available.
   */
  PipgMpc(mpc::Settings mpcSettings, multiple_shooting::Settings settings, pipg::Settings pipgSetting,
          const OptimalControlProblem& optimalControlProblem, const Initializer& initializer)
      : MPC_BASE(std::move(mpcSettings)) {
    solverPtr_.reset(new PipgMpcSolver(std::move(settings), std::move(pipgSetting), optimalControlProblem, initializer));
  };

  ~PipgMpc() override = default;

  PipgMpcSolver* getSolverPtr() override { return solverPtr_.get(); }
  const PipgMpcSolver* getSolverPtr() const override { return solverPtr_.get(); }

 protected:
  void calculateController(scalar_t initTime, const vector_t& initState, scalar_t finalTime) override {
    if (settings().coldStart_) {
      solverPtr_->reset();
    }
    solverPtr_->run(initTime, initState, finalTime);
  }

 private:
  std::unique_ptr<PipgMpcSolver> solverPtr_;
};
}  // namespace ocs2
