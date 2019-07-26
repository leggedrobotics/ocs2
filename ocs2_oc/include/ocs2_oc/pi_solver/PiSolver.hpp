#pragma once

#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/control/PiController.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_oc/oc_solver/Solver_BASE.h>
#include <ocs2_oc/pi_solver/PI_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/control/FeedforwardController.h>

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
template <size_t STATE_DIM, size_t INPUT_DIM, typename LOGIC_RULES_T = NullLogicRules>
class PiSolver final : public Solver_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = Solver_BASE<STATE_DIM, INPUT_DIM, NullLogicRules>;
  using logic_rules_t = LOGIC_RULES_T;

  using typename Base::scalar_t;
  using typename Base::scalar_array_t;
  using scalar_array2_t = std::vector<scalar_array_t>;
  using typename Base::state_vector_t;
  using  typename Base::input_vector_t;
  using  typename Base::state_matrix_t;
  using  typename Base::input_matrix_t;
  using  typename Base::input_state_matrix_t;
  using  typename Base::input_state_matrix_array_t;
  using  typename Base::state_input_matrix_t;
  using  typename Base::eigen_scalar_array_t;
  using  typename Base::cost_desired_trajectories_t;
  using  typename Base::dynamic_vector_array_t;
  using typename Base::state_vector_array_t;
  using  typename Base::state_vector_array2_t;
  using  typename Base::input_vector_array_t;
  using  typename Base::input_vector_array2_t;
  using  typename Base::controller_ptr_array_t;

  using controlled_system_base_t = ControlledSystemBase<STATE_DIM, INPUT_DIM, logic_rules_t>;
  using cost_function_t = CostFunctionBase<STATE_DIM, INPUT_DIM, logic_rules_t>;
  using rollout_t = TimeTriggeredRollout<STATE_DIM, INPUT_DIM, logic_rules_t>;
  using constraint_t = ConstraintBase<STATE_DIM, INPUT_DIM, logic_rules_t>;
  using pi_controller_t = PiController<STATE_DIM, INPUT_DIM, logic_rules_t>;

  /**
   * @brief Constructor with all options
   *
   * @param systemDynamicsPtr: System dynamics
   * @param costFunction: The cost function to optimize
   * @param constraint: Any constraints for the dynamical system
   * @param rollout_dt: The time step of the rollouts
   * @param noiseScaling The level of noise (the temperature)
   * @param numSamples How many
   */
  PiSolver(const typename controlled_system_base_t::Ptr systemDynamicsPtr, std::unique_ptr<cost_function_t> costFunction,
           const constraint_t constraint, PI_Settings piSettings)
      : settings_(std::move(piSettings)),
        systemDynamics_(systemDynamicsPtr),
        costFunction_(std::move(costFunction)),
        constraint_(constraint),
        controller_(&constraint_, costFunction_.get(), settings_.rolloutSettings_.minTimeStep_, settings_.gamma_),  //!@warn need to use member var
        numIterations_(0),
        rollout_(*systemDynamicsPtr, settings_.rolloutSettings_) {
    // TODO(jcarius) how to ensure that we are given a control affine system?
    // TODO(jcarius) how to ensure that we are given a suitable cost function?
    // TODO(jcarius) how to ensure that the constraint is input-affine and full row-rank D?

    // TODO(jcarius) enforce euler forward method in rollout and extract rollout_dt_
    // see Euler-Maruyama method (https://infoscience.epfl.ch/record/143450/files/sde_tutorial.pdf)

    auto seed = static_cast<unsigned int>(time(nullptr));
    std::cerr << "Setting random seed to controller: " << seed << std::endl;
    controller_.setRandomSeed(seed);
  }

  ~PiSolver() override = default;

  virtual void reset() override {
    costDesiredTrajectories_.clear();
    costDesiredTrajectoriesBuffer_.clear();
    nominalTimeTrajectoriesStock_.clear();
    nominalStateTrajectoriesStock_.clear();
    nominalInputTrajectoriesStock_.clear();
    nominalControllersStock_.clear();
    numIterations_ = 0;
  }

  virtual void run(const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime,
                   const scalar_array_t& partitioningTimes) override {
    numIterations_++;
    std::cerr << "mpc init state: " << initState.transpose() << std::endl;

    if (costDesiredTrajectoriesUpdated_) {
      costDesiredTrajectoriesUpdated_ = false;
      costDesiredTrajectories_.swap(costDesiredTrajectoriesBuffer_);
    }
    costFunction_->setCostDesiredTrajectories(costDesiredTrajectories_);

    const auto numSteps = static_cast<size_t>(std::round((finalTime - initTime) / settings_.rolloutSettings_.minTimeStep_)) + 1;

    // setup containers to store rollout data
    state_vector_array2_t state_vector_array2(settings_.numSamples_, state_vector_array_t(numSteps));      // vector of vectors of states
    input_vector_array2_t noisyInputVector_array2(settings_.numSamples_, input_vector_array_t(numSteps));  // vector of vectors of inputs
    scalar_array2_t stageCost(settings_.numSamples_, scalar_array_t(numSteps, 0.0));                       // vector of vectors of costs

    controller_.cacheResults_ = true;
    controller_.cacheData_.reserve(numSteps + 2);  // TODO(jcarius) check if this is the size at the end

    // -------------------------------------------------------------------------
    // forward rollout
    // -------------------------------------------------------------------------

    // a sample is a single stochastic rollout from initTime to finalTime
    for (size_t sample = 0; sample < settings_.numSamples_; sample++) {
      controller_.cacheData_.clear();

      typename rollout_t::state_vector_array_t stateTrajectory;
      typename rollout_t::scalar_array_t timeTrajectory;
      {
        // braces to guard against usage of temporary rollout quantities
        typename rollout_t::logic_rules_machine_t logicRulesMachine;
        typename rollout_t::size_array_t eventsPastTheEndIndeces;
        typename rollout_t::input_vector_array_t inputTrajectory;
        rollout_.run(0, initTime, initState, finalTime, &controller_, logicRulesMachine, timeTrajectory, eventsPastTheEndIndeces,
                     stateTrajectory, inputTrajectory);
      }

      if (controller_.cacheData_.size() != numSteps - 1) {
        throw std::runtime_error("integrator called controller too many times");
      }

      // extract cached data
      for (size_t n = 0; n < numSteps - 1; n++) {
        auto& ctrlData = controller_.cacheData_[n];

        state_vector_array2[sample][n] = ctrlData.x_;
        noisyInputVector_array2[sample][n] = ctrlData.u_;
        stageCost[sample][n] = ctrlData.stageCost_;

        if (ctrlData.t_ >= finalTime) {
          throw std::runtime_error("time is beyond final time");
        }
      }

      // final time
      state_vector_array2[sample][numSteps - 1] = stateTrajectory.back();
      controller_.computeInput(finalTime, stateTrajectory.back());
      noisyInputVector_array2[sample][numSteps - 1] = controller_.cacheData_.back().u_;
      costFunction_->setCurrentStateAndControl(finalTime, stateTrajectory.back(), input_vector_t::Zero());
      costFunction_->getTerminalCost(stageCost[sample][numSteps - 1]);
    }

    // -------------------------------------------------------------------------
    // backward pass
    // collect cost to go and calculate psi and input across all samples
    // -------------------------------------------------------------------------
    scalar_array2_t J(settings_.numSamples_, scalar_array_t(numSteps, 0.0));  // value of J (cost-to-go) for each sample and each time step
    scalar_array_t psiDistorted(numSteps, 0.0);  // value of Psi for each time step, averaged over samples (distortion = scaling of exp
                                                 // argument for numerics, no division by numSamples)
    input_vector_array_t u_opt(numSteps, input_vector_t::Zero());  // value of optimal input across time steps, averaged over samples

    // initialize J for each sample and find min across samples
    scalar_t minJ_currStep = std::numeric_limits<scalar_t>::max();
    for (size_t sample = 0; sample < settings_.numSamples_; sample++) {
      J[sample][numSteps - 1] = stageCost[sample][numSteps - 1];

      if (J[sample][numSteps - 1] < minJ_currStep) {
        minJ_currStep = J[sample][numSteps - 1];
      }
    }

    // initialize psi
    psiDistorted[numSteps - 1] =
        std::accumulate(J.begin(), J.end(), scalar_t(0.0), [this, numSteps, minJ_currStep](scalar_t a, const scalar_array_t& Ji) {
          return std::move(a) + std::exp(-(Ji[numSteps - 1] - minJ_currStep) / settings_.gamma_);
        });

    // initialize u for each sample
    for (size_t sample = 0; sample < settings_.numSamples_; sample++) {
      u_opt[numSteps - 1] +=
          noisyInputVector_array2[sample][numSteps - 1] * std::exp(-(J[sample][numSteps - 1] - minJ_currStep) / settings_.gamma_);
    }
    u_opt[numSteps - 1] /= psiDistorted[numSteps - 1];

    // propagate towards initial time
    for (int n = numSteps - 2; n >= 0; n--) {
      // calculate cost-to-go for this step for each sample
      scalar_t minJ_currStep = std::numeric_limits<scalar_t>::max();
      for (size_t sample = 0; sample < settings_.numSamples_; sample++) {
        J[sample][n] = J[sample][n + 1] + stageCost[sample][n];

        if (J[sample][n] < minJ_currStep) {
          minJ_currStep = J[sample][n];
        }
      }

      if (minJ_currStep == std::numeric_limits<scalar_t>::max()) {
        std::cerr << "cost-to-go in timestep  " << n << " is infinite for all samples." << std::endl;
        if (numIterations_ > 1) {
          return;
        } else {
          break;  // if running the first time, we have to fill time/input/state stock
        }
      }

      psiDistorted[n] = std::accumulate(J.begin(), J.end(), scalar_t(0.0), [this, n, minJ_currStep](scalar_t a, const scalar_array_t& Ji) {
        return std::move(a) + std::exp(-(Ji[n] - minJ_currStep) / settings_.gamma_);
      });

      //      if(psiDistorted[n] / numSamples_ < 0.01){
      //          std::cerr << "Warning: Less than ~1% of samples are significant in step " << n << std::endl;
      //        }

      // u_opt
      for (size_t sample = 0; sample < settings_.numSamples_; sample++) {
        u_opt[n] += noisyInputVector_array2[sample][n] * std::exp(-(J[sample][n] - minJ_currStep) / settings_.gamma_);
      }
      u_opt[n] /= psiDistorted[n];
    }

    // -------------------------------------------------------------------------
    // save data
    // -------------------------------------------------------------------------

    // time trajectory
    nominalTimeTrajectoriesStock_.clear();
    nominalTimeTrajectoriesStock_.push_back(scalar_array_t(numSteps));
    std::generate(nominalTimeTrajectoriesStock_[0].begin(), nominalTimeTrajectoriesStock_[0].end(), [n = 0, initTime, this]() mutable {
      return initTime + (n++) * settings_.rolloutSettings_.minTimeStep_;
    });

    // input trajectory
    nominalInputTrajectoriesStock_.clear();
    nominalInputTrajectoriesStock_.push_back(u_opt);
    std::cerr << "setting u_opt[0] = " << nominalInputTrajectoriesStock_[0][0] << std::endl;

    // state trajectory: perform rollout without noise but with input trajectory.
    // This also sets sampling policy for next iteration
    std::unique_ptr<ControllerBase<STATE_DIM, INPUT_DIM>> ffwCtrl(new FeedforwardController<STATE_DIM, INPUT_DIM>(nominalTimeTrajectoriesStock_[0], nominalInputTrajectoriesStock_[0]));
    controller_.setSamplingPolicy(std::move(ffwCtrl));
    controller_.gamma_ = 0.0;
    controller_.cacheResults_ = false;

    typename rollout_t::logic_rules_machine_t logicRulesMachine;
    typename rollout_t::scalar_array_t timeTrajectoryNominal;
    typename rollout_t::size_array_t eventsPastTheEndIndecesNominal;
    typename rollout_t::state_vector_array_t stateTrajectoryNominal;
    typename rollout_t::input_vector_array_t inputTrajectoryNominal;
    rollout_.run(0, initTime, initState, finalTime, &controller_, logicRulesMachine, timeTrajectoryNominal, eventsPastTheEndIndecesNominal,
                 stateTrajectoryNominal, inputTrajectoryNominal);

    nominalStateTrajectoriesStock_.clear();
    nominalStateTrajectoriesStock_.push_back(stateTrajectoryNominal);

    // controller for ROS transmission
    nominalControllersStock_.clear();
    nominalControllersStock_.emplace_back(pi_controller_t(controller_));
    updateNominalControllerPtrStock();

    // prepare local controller for next iteration
    controller_.gamma_ = settings_.gamma_;

    // debug printing
    if (settings_.debugPrint_) {
      printIterationDebug(initTime, state_vector_array2, J);

      for (size_t sample = 0; sample < settings_.numSamples_; sample++) {
        std::cerr << "sample " << sample << " initNoisyInput " << noisyInputVector_array2[sample][0].transpose() << " init cost-to-go "
                  << J[sample][0] << std::endl;
      }
    }
  }

  virtual void run(const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime,
                   const scalar_array_t& partitioningTimes, const controller_ptr_array_t& controllersStock) override {
    throw std::runtime_error("not implemented.");
  }

  virtual void blockwiseMovingHorizon(bool flag) override {
    if (flag) {
      std::cerr << "[PiSolver] BlockwiseMovingHorizon enabled." << std::endl;
    }
  }

  virtual void getPerformanceIndeces(scalar_t& costFunction, scalar_t& constraint1ISE, scalar_t& constraint2ISE) const override {
    throw std::runtime_error("not implemented.");
  }

  virtual size_t getNumIterations() const override { return numIterations_; }

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

  virtual const controller_ptr_array_t& getController() const override { return nominalControllersPtrStock_; }

  virtual void getControllerPtr(const controller_ptr_array_t*& controllersStockPtr) const override {
    controllersStockPtr = &nominalControllersPtrStock_;
  }

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

  virtual void rewindOptimizer(const size_t& firstIndex) override {}

  virtual const unsigned long long int& getRewindCounter() const override { throw std::runtime_error("not implemented."); }

  virtual const logic_rules_t* getLogicRulesPtr() const override { return &logicRules_; }

  virtual logic_rules_t* getLogicRulesPtr() override { return &logicRules_; }

  void printIterationDebug(scalar_t initTime, const state_vector_array2_t& state_vector_array2, const scalar_array2_t& J) {
    if (state_vector_array2.size() != J.size()) {
      throw std::runtime_error("printIterationDebug: Number of samples do not match");
    }

    for (int sample = 0; sample < J.size(); sample++) {
      std::cerr << "+++ Sample # " << sample << " +++ \ntime\tcost-to-go\tstateTransposed" << std::endl;
      for (int n = 0; n < J[sample].size(); n++) {
        std::cerr << initTime + n * settings_.rolloutSettings_.minTimeStep_ << "\t" << J[sample][n] << "\t" << state_vector_array2[sample][n].transpose()
                  << std::endl;
      }
    }
  }

  /**
   * @brief updates pointers in nominalControllerPtrStock from memory location of nominalControllersStock_ members
   */
  void updateNominalControllerPtrStock() {
    nominalControllersPtrStock_.clear();
    nominalControllersPtrStock_.reserve(nominalControllersStock_.size());

    for (auto& controller : nominalControllersStock_) {
      nominalControllersPtrStock_.push_back(&controller);
    }
  }

 protected:
  PI_Settings settings_;  //! path integral settings

  typename controlled_system_base_t::Ptr systemDynamics_;

  std::unique_ptr<cost_function_t> costFunction_;  //! cost function owned by solver
  constraint_t constraint_;                        //! constraint owned by solver

  pi_controller_t controller_;  //! internal controller used for rollouts

  size_t numIterations_;

  rollout_t rollout_;

  logic_rules_t logicRules_;

  cost_desired_trajectories_t costDesiredTrajectories_;  // TODO(jcarius) should this be in the base class?
  cost_desired_trajectories_t costDesiredTrajectoriesBuffer_;
  std::atomic_bool costDesiredTrajectoriesUpdated_;

  std::vector<scalar_array_t> nominalTimeTrajectoriesStock_;
  state_vector_array2_t nominalStateTrajectoriesStock_;
  input_vector_array2_t nominalInputTrajectoriesStock_;
  std::vector<pi_controller_t> nominalControllersStock_;

  controller_ptr_array_t nominalControllersPtrStock_;
};

}  // namespace ocs2
