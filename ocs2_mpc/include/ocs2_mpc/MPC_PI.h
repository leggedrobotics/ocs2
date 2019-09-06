#pragma once

#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_oc/pi_solver/PI_Settings.h>
#include <ocs2_oc/pi_solver/PiSolver.hpp>

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM>
class MPC_PI final : public MPC_BASE<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = MPC_BASE<STATE_DIM, INPUT_DIM>;
  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;

  using scalar_t = typename DIMENSIONS::scalar_t;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_vector_array2_t = typename DIMENSIONS::state_vector_array2_t;
  using input_vector_array2_t = typename DIMENSIONS::input_vector_array2_t;
  using controller_ptr_array_t = typename BASE::controller_ptr_array_t;
  using controller_const_ptr_array_t = typename BASE::controller_const_ptr_array_t;

  using solver_t = PiSolver<STATE_DIM, INPUT_DIM>;
  using constraint_t = typename solver_t::constraint_t;
  using controlled_system_base_t = typename solver_t::controlled_system_base_t;
  using cost_function_t = typename solver_t::cost_function_t;

  MPC_PI(typename controlled_system_base_t::Ptr dynamics, std::unique_ptr<cost_function_t> cost, const constraint_t constraint,
         const scalar_array_t& partitioningTimes, const MPC_Settings& mpcSettings, PI_Settings piSettings)
      : BASE(partitioningTimes, mpcSettings) {
    piSolverPtr_.reset(new solver_t(dynamics, std::move(cost), constraint, std::move(piSettings)));
    BASE::setBaseSolverPtr(piSolverPtr_.get());
  }

  virtual ~MPC_PI() = default;

  void calculateController(const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime,
                           const std::vector<scalar_array_t>*& timeTrajectoriesStockPtr,
                           const state_vector_array2_t*& stateTrajectoriesStockPtr, const input_vector_array2_t*& inputTrajectoriesStockPtr,
                           controller_const_ptr_array_t& controllersPtrStock) override {
    scalar_array_t partitioningTimesDummy;
    piSolverPtr_->run(initTime, initState, finalTime, partitioningTimesDummy);

    // get the optimized trajectories
    timeTrajectoriesStockPtr = piSolverPtr_->getOptimizedTimeTrajectoriesPtr();
    stateTrajectoriesStockPtr = piSolverPtr_->getOptimizedStateTrajectoriesPtr();
    inputTrajectoriesStockPtr = piSolverPtr_->getOptimizedInputTrajectoriesPtr();
    // get the optimal controller
    controllersPtrStock = piSolverPtr_->getOptimizedControllersPtr();
  }

  solver_t* getSolverPtr() override { return piSolverPtr_.get(); }

 protected:
  std::unique_ptr<solver_t> piSolverPtr_;
};

}  // namespace ocs2
