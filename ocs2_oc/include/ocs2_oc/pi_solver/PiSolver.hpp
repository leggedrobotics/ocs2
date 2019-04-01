#pragma once

#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/control/PiController.h>
#include <ocs2_core/cost/PathIntegralCostFunction.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_oc/oc_solver/Solver_BASE.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include <Eigen/Cholesky>
#include <random>

namespace ocs2 {

/**
 * Solver class implementing the path integral algorithm
 * initial inspiration from
 * https://github.com/vvrs/MPPIController
 * https://github.com/usc-clmc/usc-clmc-ros-pkg/blob/master/policy_learning/policy_improvement/src/policy_improvement.cpp
 * https://github.com/cbfinn/gps/blob/master/python/gps/algorithm/traj_opt/traj_opt_pi2.py
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class PiSolver : public Solver_BASE<STATE_DIM, INPUT_DIM, NullLogicRules> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = Solver_BASE<STATE_DIM, INPUT_DIM, NullLogicRules>;
  using logic_rules_t = NullLogicRules;

  using scalar_t = typename Base::scalar_t;
  using state_vector_t = typename Base::state_vector_t;
  using input_vector_t = typename Base::input_vector_t;
  using state_matrix_t = typename Base::state_matrix_t;
  using input_matrix_t = typename Base::input_matrix_t;
  using input_state_matrix_t = typename Base::input_state_matrix_t;
  using state_input_matrix_t = typename Base::state_input_matrix_t;
  using scalar_array_t = typename Base::scalar_array_t;
  using controller_array_t = typename Base::controller_array_t;
  using eigen_scalar_array_t = typename Base::eigen_scalar_array_t;
  using cost_desired_trajectories_t = typename Base::cost_desired_trajectories_t;
  using dynamic_vector_array_t = typename Base::dynamic_vector_array_t;
  using state_vector_array_t = typename Base::state_vector_array_t;
  using state_vector_array2_t = typename Base::state_vector_array2_t;
  using input_vector_array_t = typename Base::input_vector_array_t;
  using input_vector_array2_t = typename Base::input_vector_array2_t;

  using controlled_system_base_t = ControlledSystemBase<STATE_DIM, INPUT_DIM, logic_rules_t>;
  using cost_function_t = PathIntegralCostFunction<STATE_DIM, INPUT_DIM, logic_rules_t>;
  using rollout_t = TimeTriggeredRollout<STATE_DIM, INPUT_DIM, logic_rules_t>;
  using constraint_t = ConstraintBase<STATE_DIM, INPUT_DIM, logic_rules_t>;
  using controller_t = PiController<STATE_DIM, INPUT_DIM>;

  PiSolver(const typename controlled_system_base_t::Ptr systemDynamicsPtr, const typename cost_function_t::Ptr costFunction,
           const constraint_t constraint, scalar_t rollout_dt, scalar_t noiseScaling)
      : systemDynamics_(systemDynamicsPtr),
        costFunction_(costFunction),
        rollout_(*systemDynamicsPtr, Rollout_Settings(1e-9, 1e-6, 5000, rollout_dt, IntegratorType::EULER)),
        constraint_(constraint),
        rollout_dt_(rollout_dt),
        controller_(constraint, *costFunction, rollout_dt_, noiseScaling),
        gamma_(noiseScaling) {
    // TODO(jcarius) how to ensure that we are given a control affine system?
    // TODO(jcarius) how to ensure that we are given a suitable cost function?
    // TODO(jcarius) how to ensure that the constraint is input-affine and full row-rank D?

    // TODO(jcarius) enforce euler forward method in rollout and extract rollout_dt_
    // see Euler-Maruyama method (https://infoscience.epfl.ch/record/143450/files/sde_tutorial.pdf)
  }

  ~PiSolver() override = default;

  /**
   * Resets the class to its state after construction.
   */
  virtual void reset() override {
    costDesiredTrajectories_.clear();
    costDesiredTrajectoriesBuffer_.clear();
    nominalTimeTrajectoriesStock_.clear();
    nominalStateTrajectoriesStock_.clear();
    nominalInputTrajectoriesStock_.clear();
  }

  /**
   * The main routine of solver which runs the optimizer for a given initial state, initial time, and final time.
   *
   * @param [in] initTime: The initial time.
   * @param [in] initState: The initial state.
   * @param [in] finalTime: The final time.
   * @param [in] partitioningTimes: The partitioning times between subsystems.
   */
  virtual void run(const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime,
                   const scalar_array_t& partitioningTimes) override {
    if (costDesiredTrajectoriesUpdated_) {
      costDesiredTrajectoriesUpdated_ = false;
      costDesiredTrajectories_.swap(costDesiredTrajectoriesBuffer_);
    }
    costFunction_->setCostDesiredTrajectories(costDesiredTrajectories_);

    constexpr size_t numSamples = 10;  //! number of random walks to be sampled
    const auto numSteps = static_cast<size_t>(std::floor((finalTime - initTime) / rollout_dt_));

    state_vector_array2_t state_vector_array2(numSamples, state_vector_array_t(numSteps));  //! vector of vectors of states
    scalar_array_t trajectoryCostVtilde(numSamples, scalar_t(0.0));

    scalar_t psi(0.0);  // value of Psi(x,t) for x=initState, t=init_time
    input_vector_t u_opt = input_vector_t::Zero();

    for (size_t sample = 0; sample < numSamples; sample++) {
      state_vector_array2[sample][0] = initState;
      controller_.computeInput(initTime, initState);

      input_vector_t noiseOnDxDtAtTimeZero;

      for (size_t n = 0; n < numSteps - 1; n++) {
        // calculate costs
        trajectoryCostVtilde[sample] +=
            controller_.V_ + 0.5 * (controller_.Ddagger_ * controller_.c_).dot(controller_.R_ * controller_.Ddagger_ * controller_.c_);
        trajectoryCostVtilde[sample] -=
            0.5 * controller_.r_.transpose() * (input_matrix_t::Identity() - controller_.Dtilde_) * controller_.Rinv_ * controller_.r_;
        trajectoryCostVtilde[sample] -= (controller_.Ddagger_ * controller_.c_).transpose() * controller_.r_;

        const auto stepTime = initTime + rollout_dt_ * n;
        typename rollout_t::logic_rules_machine_t logicRulesMachine;
        typename rollout_t::scalar_array_t timeTrajectory;
        typename rollout_t::size_array_t eventsPastTheEndIndeces;
        typename rollout_t::state_vector_array_t stateTrajectory;
        typename rollout_t::input_vector_array_t inputTrajectory;

        // step forward in time
        rollout_.run(0, stepTime, state_vector_array2[sample][n], stepTime + rollout_dt_, &controller_, logicRulesMachine, timeTrajectory,
                     eventsPastTheEndIndeces, stateTrajectory, inputTrajectory);

        if (n == 0) {
          noiseOnDxDtAtTimeZero = controller_.noiseInput_;  // TODO(jcarius) is there a dt missing?
        }
      }
      scalar_t terminalCost;
      costFunction_->setCurrentStateAndControl(finalTime, state_vector_array2[sample][numSteps - 1], input_vector_t::Zero());
      costFunction_->getTerminalCost(terminalCost);
      trajectoryCostVtilde[sample] += terminalCost;

      // TODO(jcarius) check how to implement this numerically stable
      psi += std::exp(-trajectoryCostVtilde[sample] / gamma_);
      u_opt += noiseOnDxDtAtTimeZero * std::exp(-trajectoryCostVtilde[sample] / gamma_);
    }

    psi /= numSamples;
    u_opt /= numSamples * psi;

    // assign solution to member variables
    nominalTimeTrajectoriesStock_.clear();
    nominalTimeTrajectoriesStock_.push_back(scalar_array_t{initTime});

    nominalStateTrajectoriesStock_.clear();
    nominalStateTrajectoriesStock_.push_back(state_vector_array_t{initState});

    nominalInputTrajectoriesStock_.clear();
    nominalInputTrajectoriesStock_.push_back(input_vector_array_t{u_opt});
  }

  /**
   * The main routine of solver which runs the optimizer for a given initial state, initial time, final time, and
   * initial controller.
   *
   * @param [in] initTime: The initial time.
   * @param [in] initState: The initial state.
   * @param [in] finalTime: The final time.
   * @param [in] partitioningTimes: The time partitioning.
   * @param [in] controllersStock: Array of the initial control policies.
   * @param [in] costDesiredTrajectories: The cost desired trajectories.
   */
  virtual void run(const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime,
                   const scalar_array_t& partitioningTimes, const controller_array_t& controllersStock) override {
    throw std::runtime_error("not implemented.");
  }

  /**
   * MPC_BASE activates this if the final time of the MPC will increase by the length of a time partition instead
   * of commonly used scheme where the final time is gradually increased.
   *
   * @param [in] flag: If set true, the final time of the MPC will increase by the length of a time partition.
   */
  virtual void blockwiseMovingHorizon(bool flag) override {
    if (flag) {
      std::cout << "[PiSolver] BlockwiseMovingHorizon enabled." << std::endl;
    }
  }

  /**
   * Gets the cost function and ISEs of the type-1 and type-2 constraints at the initial time.
   *
   * @param [out] costFunction: cost function value
   * @param [out] constraint1ISE: type-1 constraint ISE.
   * @param [out] constraint1ISE: type-2 constraint ISE.
   */
  virtual void getPerformanceIndeces(scalar_t& costFunction, scalar_t& constraint1ISE, scalar_t& constraint2ISE) const override {
    throw std::runtime_error("not implemented.");
  }
  /**
   * Gets number of iterations.
   *
   * @return Number of iterations.
   */
  virtual size_t getNumIterations() const override {
    throw std::runtime_error("not implemented.");
    return 0;
  }

  /**
   * Gets iterations Log of SLQ.
   *
   * @param [out] iterationCost: Each iteration's cost.
   * @param [out] iterationISE1: Each iteration's type-1 constraints ISE.
   * @param [out] iterationISE2: Each iteration's type-2 constraints ISE.
   */
  virtual void getIterationsLog(eigen_scalar_array_t& iterationCost, eigen_scalar_array_t& iterationISE1,
                                eigen_scalar_array_t& iterationISE2) const override {
    throw std::runtime_error("not implemented.");
  }

  /**
   * Gets Iterations Log of SLQ
   *
   * @param [out] iterationCostPtr: A pointer to each iteration's cost.
   * @param [out] iterationISE1Ptr: A pointer to each iteration's type-1 constraints ISE.
   * @param [out] iterationISE2Ptr: A pointer to each iteration's type-2 constraints ISE.
   */
  virtual void getIterationsLogPtr(const eigen_scalar_array_t*& iterationCostPtr, const eigen_scalar_array_t*& iterationISE1Ptr,
                                   const eigen_scalar_array_t*& iterationISE2Ptr) const override {
    throw std::runtime_error("not implemented.");
  }

  /**
   * Gets final time of optimization
   *
   * @return finalTime
   */
  virtual const scalar_t& getFinalTime() const override { throw std::runtime_error("not implemented."); }

  /**
   * Returns the final time of optimization.
   *
   * @return finalTime
   */
  virtual const scalar_array_t& getPartitioningTimes() const override { throw std::runtime_error("not implemented."); }

  /**
   * Gets the cost function desired trajectories.
   *
   * @param [out] costDesiredTrajectories: A pointer to the cost function desired trajectories
   */
  virtual void getCostDesiredTrajectoriesPtr(const cost_desired_trajectories_t*& costDesiredTrajectoriesPtr) const override {
    costDesiredTrajectoriesPtr = &costDesiredTrajectories_;
  }

  /**
   * Sets the cost function desired trajectories.
   *
   * @param [in] costDesiredTrajectories: The cost function desired trajectories
   */
  virtual void setCostDesiredTrajectories(const cost_desired_trajectories_t& costDesiredTrajectories) override {
    costDesiredTrajectoriesBuffer_ = costDesiredTrajectories;
    costDesiredTrajectoriesUpdated_ = true;
  }

  /**
   * Sets the cost function desired trajectories.
   *
   * @param [in] desiredTimeTrajectory: The desired time trajectory for cost.
   * @param [in] desiredStateTrajectory: The desired state trajectory for cost.
   * @param [in] desiredInputTrajectory: The desired input trajectory for cost.
   */
  virtual void setCostDesiredTrajectories(const scalar_array_t& desiredTimeTrajectory, const dynamic_vector_array_t& desiredStateTrajectory,
                                          const dynamic_vector_array_t& desiredInputTrajectory) override {
    throw std::runtime_error("not implemented.");
  }

  /**
   * Swaps the cost function desired trajectories.
   *
   * @param [in] costDesiredTrajectories: The cost function desired trajectories
   */
  virtual void swapCostDesiredTrajectories(cost_desired_trajectories_t& costDesiredTrajectories) override {
    costDesiredTrajectoriesBuffer_.swap(costDesiredTrajectories);
    costDesiredTrajectoriesUpdated_ = true;
  }

  /**
   * Swaps the cost function desired trajectories.
   *
   * @param [in] desiredTimeTrajectory: The desired time trajectory for cost.
   * @param [in] desiredStateTrajectory: The desired state trajectory for cost.
   * @param [in] desiredInputTrajectory: The desired input trajectory for cost.
   */
  virtual void swapCostDesiredTrajectories(scalar_array_t& desiredTimeTrajectory, dynamic_vector_array_t& desiredStateTrajectory,
                                           dynamic_vector_array_t& desiredInputTrajectory) override {
    throw std::runtime_error("not implemented.");
  }

  /**
   * Whether the cost function desired trajectories is updated.
   *
   * @return true if it is updated.
   */
  virtual bool costDesiredTrajectoriesUpdated() const override { throw std::runtime_error("not implemented."); }

  /**
   * Returns the optimal array of the control policies.
   *
   * @return controllersStock: The optimal array of the control policies.
   */
  virtual const controller_array_t& getController() const override { throw std::runtime_error("not implemented."); }

  /**
   * Gets a pointer to the optimal array of the control policies.
   *
   * @param [out] controllersStockPtr: A pointer to the optimal array of the control policies
   */
  virtual void getControllerPtr(const controller_array_t*& controllersStockPtr) const override {
    throw std::runtime_error("not implemented.");
  }

  /**
   * Swaps the output array of the control policies with the nominal one.
   * Care should be take since this method modifies the internal variable.
   *
   * @param [out] controllersStock: A reference to the optimal array of the control policies
   */
  virtual void swapController(controller_array_t& controllersStock) override { throw std::runtime_error("not implemented."); }

  /**
   * Returns the nominal time trajectories.
   *
   * @return nominalTimeTrajectoriesStock: Array of trajectories containing the output time trajectory stamp.
   */
  virtual const std::vector<scalar_array_t>& getNominalTimeTrajectories() const override { throw std::runtime_error("not implemented."); }

  /**
   * Returns the nominal state trajectories.
   *
   * @return nominalStateTrajectoriesStock: Array of trajectories containing the output state trajectory.
   */
  virtual const state_vector_array2_t& getNominalStateTrajectories() const override { throw std::runtime_error("not implemented."); }

  /**
   * Returns the nominal input trajectories.
   *
   * @return nominalInputTrajectoriesStock: Array of trajectories containing the output control input trajectory.
   */
  virtual const input_vector_array2_t& getNominalInputTrajectories() const override { throw std::runtime_error("not implemented."); }

  /**
   * Gets a pointer to the nominal time, state, and input trajectories.
   *
   * @param [out] nominalTimeTrajectoriesStockPtr: A pointer to an array of trajectories containing the output time trajectory stamp.
   * @param [out] nominalStateTrajectoriesStockPtr: A pointer to an array of trajectories containing the output state trajectory.
   * @param [out] nominalInputTrajectoriesStockPtr: A pointer to an array of trajectories containing the output control input trajectory.
   */
  virtual void getNominalTrajectoriesPtr(const std::vector<scalar_array_t>*& nominalTimeTrajectoriesStockPtr,
                                         const state_vector_array2_t*& nominalStateTrajectoriesStockPtr,
                                         const input_vector_array2_t*& nominalInputTrajectoriesStockPtr) const override {
    // TODO(jcarius) passing out raw pointers to member variables (!!)
    nominalTimeTrajectoriesStockPtr = &nominalTimeTrajectoriesStock_;
    nominalStateTrajectoriesStockPtr = &nominalStateTrajectoriesStock_;
    nominalInputTrajectoriesStockPtr = &nominalInputTrajectoriesStock_;
  }

  /**
   * Swaps the the outputs with the nominal trajectories.
   * Care should be take since this method modifies the internal variable.
   *
   * @param [out] nominalTimeTrajectoriesStock: Array of trajectories containing the output time trajectory stamp.
   * @param [out] nominalStateTrajectoriesStock: Array of trajectories containing the output state trajectory.
   * @param [out] nominalInputTrajectoriesStock: Array of trajectories containing the output control input trajectory.
   */
  virtual void swapNominalTrajectories(std::vector<scalar_array_t>& nominalTimeTrajectoriesStock,
                                       state_vector_array2_t& nominalStateTrajectoriesStock,
                                       input_vector_array2_t& nominalInputTrajectoriesStock) override {
    throw std::runtime_error("not implemented.");
  }

  /**
   * Rewinds optimizer internal variables.
   *
   * @param [in] firstIndex: The index which we want to rewind to.
   */
  virtual void rewindOptimizer(const size_t& firstIndex) override { throw std::runtime_error("not implemented."); }

  /**
   * Get rewind counter.
   *
   * @return Number of partition rewinds since construction of the class.
   */
  virtual const unsigned long long int& getRewindCounter() const override { throw std::runtime_error("not implemented."); }

 protected:
  typename controlled_system_base_t::Ptr systemDynamics_;
  typename cost_function_t::Ptr costFunction_;

  constraint_t constraint_;

  controller_t controller_;

  rollout_t rollout_;
  scalar_t rollout_dt_;  //! time step size of Euler integration rollout
  scalar_t gamma_;       //! scaling of noise (~temperature)

  cost_desired_trajectories_t costDesiredTrajectories_;  // TODO(jcarius) should this be in the base class?
  cost_desired_trajectories_t costDesiredTrajectoriesBuffer_;
  std::atomic_bool costDesiredTrajectoriesUpdated_;

  std::vector<scalar_array_t> nominalTimeTrajectoriesStock_;
  state_vector_array2_t nominalStateTrajectoriesStock_;
  input_vector_array2_t nominalInputTrajectoriesStock_;
};  // namespace ocs2
}  // namespace ocs2
