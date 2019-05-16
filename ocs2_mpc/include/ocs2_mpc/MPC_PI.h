#pragma once

#include <ocs2_mpc/MPC_BASE.h>

#include <ocs2_core/cost/PathIntegralCostFunction.h>
#include <ocs2_oc/pi_solver/PiSolver.hpp>

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T = NullLogicRules>
class MPC_PI : public MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<MPC_PI<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>> Ptr;

  typedef MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> BASE;
  typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;

  typedef typename DIMENSIONS::scalar_t scalar_t;
  typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
  typedef typename DIMENSIONS::state_vector_t state_vector_t;
  typedef typename DIMENSIONS::state_vector_array2_t state_vector_array2_t;
  typedef typename DIMENSIONS::input_vector_array2_t input_vector_array2_t;
  typedef typename BASE::controller_ptr_array_t controller_ptr_array_t;

  typedef Solver_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> solver_base_t;
  typedef PiSolver<STATE_DIM, INPUT_DIM> solver_t;
  typedef typename solver_t::controlled_system_base_t dynamics_t;
  typedef typename solver_t::cost_function_t cost_t;
  typedef typename solver_t::constraint_t constraint_t;

  MPC_PI(typename dynamics_t::Ptr dynamics, std::unique_ptr<cost_t> cost, const constraint_t constraint, scalar_t rollout_dt,
         scalar_t noiseScaling, const scalar_array_t& partitioningTimes, const MPC_Settings& settings)
      : BASE(partitioningTimes, settings) {
    piSolverPtr_.reset(new solver_t(dynamics, std::move(cost), constraint, rollout_dt, noiseScaling));
    BASE::setBaseSolverPtr(piSolverPtr_.get());
  }

  virtual ~MPC_PI() = default;

  /**
   * Solves the optimal control problem for the given state and time period ([initTime,finalTime]).
   *
   * @param [out] initTime: Initial time. This value can be adjusted by the optimizer.
   * @param [in] initState: Initial state.
   * @param [out] finalTime: Final time. This value can be adjusted by the optimizer.
   * @param [out] timeTrajectoriesStockPtr: A pointer to the optimized time trajectories.
   * @param [out] stateTrajectoriesStockPtr: A pointer to the optimized state trajectories.
   * @param [out] inputTrajectoriesStockPtr: A pointer to the optimized input trajectories.
   * @param [out] controllerStockPtr: A pointer to the optimized control policy.
   */
  virtual void calculateController(const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime,
                                   const std::vector<scalar_array_t>*& timeTrajectoriesStockPtr,
                                   const state_vector_array2_t*& stateTrajectoriesStockPtr,
                                   const input_vector_array2_t*& inputTrajectoriesStockPtr,
                                   const controller_ptr_array_t*& controllerStockPtr) override {
    scalar_array_t partitioningTimesDummy;
    piSolverPtr_->run(initTime, initState, finalTime, partitioningTimesDummy);

    piSolverPtr_->getNominalTrajectoriesPtr(timeTrajectoriesStockPtr, stateTrajectoriesStockPtr, inputTrajectoriesStockPtr);
    piSolverPtr_->getControllerPtr(controllerStockPtr);
  }

  /**
   * Gets a pointer to the underlying solver used in the MPC.
   *
   * @return A pointer to the underlying solver used in the MPC
   */
  virtual solver_base_t* getSolverPtr() override { return piSolverPtr_.get(); }  // TODO(jcarius) should this happen in the base class?

 protected:
  std::unique_ptr<solver_t> piSolverPtr_;
};

}  // namespace ocs2
