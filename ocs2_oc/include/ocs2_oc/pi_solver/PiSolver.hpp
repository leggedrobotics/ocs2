#pragma once

#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/control/LinearController.h>
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
  using scalar_array_t = typename Base::scalar_array_t;
  using scalar_array2_t = std::vector<scalar_array_t>;
  using state_vector_t = typename Base::state_vector_t;
  using input_vector_t = typename Base::input_vector_t;
  using state_matrix_t = typename Base::state_matrix_t;
  using input_matrix_t = typename Base::input_matrix_t;
  using input_state_matrix_t = typename Base::input_state_matrix_t;
  using input_state_matrix_array_t = typename Base::input_state_matrix_array_t;
  using state_input_matrix_t = typename Base::state_input_matrix_t;
  using eigen_scalar_array_t = typename Base::eigen_scalar_array_t;
  using cost_desired_trajectories_t = typename Base::cost_desired_trajectories_t;
  using dynamic_vector_array_t = typename Base::dynamic_vector_array_t;
  using state_vector_array_t = typename Base::state_vector_array_t;
  using state_vector_array2_t = typename Base::state_vector_array2_t;
  using input_vector_array_t = typename Base::input_vector_array_t;
  using input_vector_array2_t = typename Base::input_vector_array2_t;

  using controller_ptr_array_t = typename Base::controller_ptr_array_t;
  using controlled_system_base_t = ControlledSystemBase<STATE_DIM, INPUT_DIM, logic_rules_t>;
  using cost_function_t = PathIntegralCostFunction<STATE_DIM, INPUT_DIM, logic_rules_t>;
  using rollout_t = TimeTriggeredRollout<STATE_DIM, INPUT_DIM, logic_rules_t>;
  using constraint_t = ConstraintBase<STATE_DIM, INPUT_DIM, logic_rules_t>;
  using pi_controller_t = PiController<STATE_DIM, INPUT_DIM>;

  PiSolver(const typename controlled_system_base_t::Ptr systemDynamicsPtr, const typename cost_function_t::Ptr costFunction,
           const constraint_t constraint, scalar_t rollout_dt, scalar_t noiseScaling)
      : systemDynamics_(systemDynamicsPtr),
        costFunction_(costFunction),
        rollout_(*systemDynamicsPtr, Rollout_Settings(1e-9, 1e-6, 5000, rollout_dt, IntegratorType::EULER)),
        constraint_(constraint),
        rollout_dt_(rollout_dt),
        controller_(constraint, *costFunction, rollout_dt, noiseScaling),
        gamma_(noiseScaling) {
    // TODO(jcarius) how to ensure that we are given a control affine system?
    // TODO(jcarius) how to ensure that we are given a suitable cost function?
    // TODO(jcarius) how to ensure that the constraint is input-affine and full row-rank D?

    // TODO(jcarius) enforce euler forward method in rollout and extract rollout_dt_
    // see Euler-Maruyama method (https://infoscience.epfl.ch/record/143450/files/sde_tutorial.pdf)
  }

  ~PiSolver() override = default;

  virtual void reset() override {
    costDesiredTrajectories_.clear();
    costDesiredTrajectoriesBuffer_.clear();
    nominalTimeTrajectoriesStock_.clear();
    nominalStateTrajectoriesStock_.clear();
    nominalInputTrajectoriesStock_.clear();
    nominalControllersStock_.clear();
  }

  virtual void run(const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime,
                   const scalar_array_t& partitioningTimes) override {
    if (costDesiredTrajectoriesUpdated_) {
      costDesiredTrajectoriesUpdated_ = false;
      costDesiredTrajectories_.swap(costDesiredTrajectoriesBuffer_);
    }
    costFunction_->setCostDesiredTrajectories(costDesiredTrajectories_);

    constexpr size_t numSamples = 10;  // number of random walks to be sampled
    const auto numSteps = static_cast<size_t>(std::floor((finalTime - initTime) / rollout_dt_));

    // setup containers to store rollout data
    state_vector_array2_t state_vector_array2(numSamples, state_vector_array_t(numSteps));      // vector of vectors of states
    input_vector_array2_t noiseInputVector_array2(numSamples, input_vector_array_t(numSteps));  // vector of vectors of inputs
    scalar_array2_t costVtilde(numSamples, scalar_array_t(numSteps, 0.0));                      // vector of vectors of costs

    // -------------------------------------------------------------------------
    // forward rollout
    // -------------------------------------------------------------------------

    // a sample is a single stochastic rollout from initTime to finalTime
    for (size_t sample = 0; sample < numSamples; sample++) {
      state_vector_array2[sample][0] = initState;
      controller_.computeInput(initTime, initState);

      // stepping through time to avoid recomputing quantities that the controller already computed
      for (size_t n = 0; n < numSteps - 1; n++) {
        // calculate costs
        costVtilde[sample][n] =
            controller_.V_ + 0.5 * (controller_.Ddagger_ * controller_.c_).dot(controller_.R_ * controller_.Ddagger_ * controller_.c_);
        costVtilde[sample][n] -=
            0.5 * controller_.r_.transpose() * (input_matrix_t::Identity() - controller_.Dtilde_) * controller_.Rinv_ * controller_.r_;
        costVtilde[sample][n] -= (controller_.Ddagger_ * controller_.c_).transpose() * controller_.r_;

        const auto currStepTime = initTime + rollout_dt_ * n;
        typename rollout_t::logic_rules_machine_t logicRulesMachine;
        typename rollout_t::scalar_array_t timeTrajectory;
        typename rollout_t::size_array_t eventsPastTheEndIndeces;
        typename rollout_t::state_vector_array_t stateTrajectory;
        typename rollout_t::input_vector_array_t inputTrajectory;

        // step forward in time
        rollout_.run(0, currStepTime, state_vector_array2[sample][n], currStepTime + rollout_dt_, &controller_, logicRulesMachine,
                     timeTrajectory, eventsPastTheEndIndeces, stateTrajectory, inputTrajectory);

        // extract results of step
        if (timeTrajectory.size() != 2) {
          throw std::runtime_error("Expected rollout to do a single step only.");
        }
        state_vector_array2[sample][n + 1] = stateTrajectory.back();
        noiseInputVector_array2[sample][n] = controller_.noiseInput_;  // TODO(jcarius) is there a dt missing?
      }
      scalar_t terminalCost;
      costFunction_->setCurrentStateAndControl(finalTime, state_vector_array2[sample][numSteps - 1], input_vector_t::Zero());
      costFunction_->getTerminalCost(terminalCost);
      costVtilde[sample][numSteps - 1] = terminalCost;
    }

    // make a single rollout without noise to save as nominal state trajectory later
controller_.gamma_ = 0.0;
typename rollout_t::logic_rules_machine_t logicRulesMachine;
typename rollout_t::scalar_array_t timeTrajectoryNominal;
typename rollout_t::size_array_t eventsPastTheEndIndecesNominal;
typename rollout_t::state_vector_array_t stateTrajectoryNominal;
typename rollout_t::input_vector_array_t inputTrajectoryNominal;
rollout_.run(0, initTime, initState, finalTime, &controller_, logicRulesMachine, timeTrajectoryNominal, eventsPastTheEndIndecesNominal, stateTrajectoryNominal, inputTrajectoryNominal);

controller_.gamma_ = gamma_;

    // -------------------------------------------------------------------------
    // backward pass
    // collect cost to go and calculate psi and input across all samples
    // -------------------------------------------------------------------------
    scalar_array2_t J(numSamples, scalar_array_t(numSteps, 0.0));      // value of J (cost-to-go) across time steps for each sample
    scalar_array_t psi(numSteps, 0.0);                                 // value of Psi across time steps, averaged over samples
    input_vector_array_t u_opt(numSteps, input_vector_t::Zero());  // value of optimal input across time steps, averaged over samples

    // initialize cost-to-go for each sample
    for (size_t sample = 0; sample < numSamples; sample++) {
      J[sample][numSteps - 1] = costVtilde[sample][numSteps - 1];
    }

    // initialize psi
    psi[numSteps - 1] =
        std::accumulate(J.begin(), J.end(), scalar_t(0.0),
                        [this, numSteps](const scalar_t& a, const scalar_array_t& b) { return a + std::exp((-1.0 / gamma_) * b[numSteps - 1]); }) /
        numSamples;

    // initialize u
    for (size_t sample = 0; sample < numSamples; sample++) {
        u_opt[numSteps-1] += noiseInputVector_array2[sample][numSteps-1] * std::exp((-1.0 / gamma_) * J[sample][numSteps - 1]);
    }
    u_opt[numSteps-1] /= numSamples * psi[numSteps - 1];

    // propagate towards initial time
    for (int n = numSteps - 2; n >= 0; n--) {
      // calculate cost-to-go for this step for each sample
      for (size_t sample = 0; sample < numSamples; sample++) {
        J[sample][n] = J[sample][n + 1] + costVtilde[sample][n];
      }

      psi[n] = std::accumulate(J.begin(), J.end(), scalar_t(0.0),
                               [this,n](const scalar_t& a, const scalar_array_t& b) { return a + std::exp((-1.0 / gamma_) * b[n]); }) /
               numSamples;

      // u_opt
      for (size_t sample = 0; sample < numSamples; sample++) {
          u_opt[n] += noiseInputVector_array2[sample][n] * std::exp((-1.0 / gamma_) * J[sample][n]);
      }
      u_opt[n] /= numSamples * psi[n];
    }


    // assign solution to member variables
    nominalTimeTrajectoriesStock_.clear();
    nominalTimeTrajectoriesStock_.push_back(scalar_array_t(numSteps));
    std::generate(nominalTimeTrajectoriesStock_[0].begin(),nominalTimeTrajectoriesStock_[0].end(),
            [tt = initTime, this]() mutable {tt += rollout_dt_; return tt;});

    nominalStateTrajectoriesStock_.clear();
    nominalStateTrajectoriesStock_.push_back(stateTrajectoryNominal);

    nominalInputTrajectoriesStock_.clear();
    nominalInputTrajectoriesStock_.push_back(u_opt);

    nominalControllersStock_.clear();
    nominalControllersStock_.push_back(pi_controller_t(constraint_, *costFunction_, rollout_dt_, 0.0));
    nominalControllersStock_.back().setFeedforwardInputAndState(nominalTimeTrajectoriesStock_[0], nominalStateTrajectoriesStock_[0], nominalInputTrajectoriesStock_[0]);
    updateNominalControllerPtrStock();
  }

  virtual void run(const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime,
                   const scalar_array_t& partitioningTimes, const controller_ptr_array_t& controllersStock) override {
    throw std::runtime_error("not implemented.");
  }

  virtual void blockwiseMovingHorizon(bool flag) override {
    if (flag) {
      std::cout << "[PiSolver] BlockwiseMovingHorizon enabled." << std::endl;
    }
  }

  virtual void getPerformanceIndeces(scalar_t& costFunction, scalar_t& constraint1ISE, scalar_t& constraint2ISE) const override {
    throw std::runtime_error("not implemented.");
  }

  virtual size_t getNumIterations() const override {
    throw std::runtime_error("not implemented.");
    return 0;
  }

  virtual void getIterationsLog(eigen_scalar_array_t& iterationCost, eigen_scalar_array_t& iterationISE1,
                                eigen_scalar_array_t& iterationISE2) const override {
    throw std::runtime_error("not implemented.");
  }


  virtual void getIterationsLogPtr(const eigen_scalar_array_t*& iterationCostPtr, const eigen_scalar_array_t*& iterationISE1Ptr,
                                   const eigen_scalar_array_t*& iterationISE2Ptr) const override {
    throw std::runtime_error("not implemented.");
  }


  virtual const scalar_t& getFinalTime() const override { throw std::runtime_error("not implemented."); }

  /**
   * Returns the final time of optimization.
   *
   * @return finalTime
   */
  virtual const scalar_array_t& getPartitioningTimes() const override { throw std::runtime_error("not implemented."); }


  virtual void getCostDesiredTrajectoriesPtr(const cost_desired_trajectories_t*& costDesiredTrajectoriesPtr) const override {
    costDesiredTrajectoriesPtr = &costDesiredTrajectories_;
  }


  virtual void setCostDesiredTrajectories(const cost_desired_trajectories_t& costDesiredTrajectories) override {
    costDesiredTrajectoriesBuffer_ = costDesiredTrajectories;
    costDesiredTrajectoriesUpdated_ = true;
  }


  virtual void setCostDesiredTrajectories(const scalar_array_t& desiredTimeTrajectory, const dynamic_vector_array_t& desiredStateTrajectory,
                                          const dynamic_vector_array_t& desiredInputTrajectory) override {
    throw std::runtime_error("not implemented.");
  }

  virtual void swapCostDesiredTrajectories(cost_desired_trajectories_t& costDesiredTrajectories) override {
    costDesiredTrajectoriesBuffer_.swap(costDesiredTrajectories);
    costDesiredTrajectoriesUpdated_ = true;
  }


  virtual void swapCostDesiredTrajectories(scalar_array_t& desiredTimeTrajectory, dynamic_vector_array_t& desiredStateTrajectory,
                                           dynamic_vector_array_t& desiredInputTrajectory) override {
    throw std::runtime_error("not implemented.");
  }


  virtual bool costDesiredTrajectoriesUpdated() const override { throw std::runtime_error("not implemented."); }


  virtual const controller_ptr_array_t& getController() const override {
      return nominalControllersPtrStock_; }


  virtual void getControllerPtr(const controller_ptr_array_t*& controllersStockPtr) const override {
      controllersStockPtr = &nominalControllersPtrStock_;
  }

  virtual void swapController(controller_ptr_array_t& controllersStock) override { throw std::runtime_error("not implemented."); }

  virtual const std::vector<scalar_array_t>& getNominalTimeTrajectories() const override { throw std::runtime_error("not implemented."); }

  virtual const state_vector_array2_t& getNominalStateTrajectories() const override { throw std::runtime_error("not implemented."); }

  virtual const input_vector_array2_t& getNominalInputTrajectories() const override { throw std::runtime_error("not implemented."); }

  virtual void getNominalTrajectoriesPtr(const std::vector<scalar_array_t>*& nominalTimeTrajectoriesStockPtr,
                                         const state_vector_array2_t*& nominalStateTrajectoriesStockPtr,
                                         const input_vector_array2_t*& nominalInputTrajectoriesStockPtr) const override {
    // TODO(jcarius) passing out raw pointers to member variables (!!)
    nominalTimeTrajectoriesStockPtr = &nominalTimeTrajectoriesStock_;
    nominalStateTrajectoriesStockPtr = &nominalStateTrajectoriesStock_;
    nominalInputTrajectoriesStockPtr = &nominalInputTrajectoriesStock_;
  }


  virtual void swapNominalTrajectories(std::vector<scalar_array_t>& nominalTimeTrajectoriesStock,
                                       state_vector_array2_t& nominalStateTrajectoriesStock,
                                       input_vector_array2_t& nominalInputTrajectoriesStock) override {
    throw std::runtime_error("not implemented.");
  }


  virtual void rewindOptimizer(const size_t& firstIndex) override { throw std::runtime_error("not implemented."); }


  virtual const unsigned long long int& getRewindCounter() const override { throw std::runtime_error("not implemented."); }

  /**
   * @brief updates pointers in nominalControllerPtrStock from memory location of nominalControllersStock_ members
   */
  void updateNominalControllerPtrStock(){
      nominalControllersPtrStock_.clear();
      nominalControllersPtrStock_.reserve(nominalControllersStock_.size());

      for(auto& controller : nominalControllersStock_){
          nominalControllersPtrStock_.push_back(&controller);
      }
  }

 protected:
  typename controlled_system_base_t::Ptr systemDynamics_;
  typename cost_function_t::Ptr costFunction_;

  constraint_t constraint_;

  pi_controller_t controller_;

  rollout_t rollout_;
  scalar_t rollout_dt_;  //! time step size of Euler integration rollout
  scalar_t gamma_;       //! scaling of noise (~temperature)

  cost_desired_trajectories_t costDesiredTrajectories_;  // TODO(jcarius) should this be in the base class?
  cost_desired_trajectories_t costDesiredTrajectoriesBuffer_;
  std::atomic_bool costDesiredTrajectoriesUpdated_;

  std::vector<scalar_array_t> nominalTimeTrajectoriesStock_;
  state_vector_array2_t nominalStateTrajectoriesStock_;
  input_vector_array2_t nominalInputTrajectoriesStock_;
  std::vector<pi_controller_t> nominalControllersStock_;

  controller_ptr_array_t nominalControllersPtrStock_;
};  // namespace ocs2
}  // namespace ocs2
