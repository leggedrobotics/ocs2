#pragma once

#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/control/PiController.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_oc/oc_solver/Solver_BASE.h>
#include <ocs2_oc/pi_solver/PI_Settings.h>
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
class PiSolver final : public Solver_BASE<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = Solver_BASE<STATE_DIM, INPUT_DIM>;

  using typename Base::controller_ptr_array_t;
  using typename Base::cost_desired_trajectories_t;
  using typename Base::dynamic_vector_array_t;
  using typename Base::dynamic_vector_t;
  using typename Base::eigen_scalar_array_t;
  using typename Base::feedforward_controller_t;
  using typename Base::input_matrix_t;
  using typename Base::input_state_matrix_array_t;
  using typename Base::input_state_matrix_t;
  using typename Base::input_vector_array2_t;
  using typename Base::input_vector_array_t;
  using typename Base::input_vector_t;
  using typename Base::primal_solution_t;
  using typename Base::scalar_array_t;
  using typename Base::scalar_t;
  using typename Base::state_input_matrix_t;
  using typename Base::state_matrix_t;
  using typename Base::state_vector_array2_t;
  using typename Base::state_vector_array_t;
  using typename Base::state_vector_t;

  using scalar_array2_t = std::vector<scalar_array_t>;
  using controlled_system_base_t = ControlledSystemBase<STATE_DIM, INPUT_DIM>;
  using cost_function_t = CostFunctionBase<STATE_DIM, INPUT_DIM>;
  using rollout_t = TimeTriggeredRollout<STATE_DIM, INPUT_DIM>;
  using constraint_t = ConstraintBase<STATE_DIM, INPUT_DIM>;
  using pi_controller_t = PiController<STATE_DIM, INPUT_DIM>;

  /**
   * @brief Constructor with all options
   *
   * @param systemDynamicsPtr: System dynamics
   * @param costFunction: The cost function to optimize
   * @param constraint: Any constraints for the dynamical system
   * @param piSettings: Settings related to PI algorithm
   * @param logicRules: Optional pointer to logic rules
   */
  PiSolver(const typename controlled_system_base_t::Ptr systemDynamicsPtr, std::unique_ptr<cost_function_t> costFunction,
           const constraint_t constraint, PI_Settings piSettings, std::shared_ptr<HybridLogicRules> logicRules = nullptr)
      : Base(std::move(logicRules)),
        piSettings_(std::move(piSettings)),
        systemDynamics_(systemDynamicsPtr),
        costFunction_(std::move(costFunction)),
        constraint_(constraint),
        controller_(&constraint_, costFunction_.get(), piSettings_.rolloutSettings_.minTimeStep_,
                    piSettings_.gamma_),  //!@warn need to use member var
        numIterations_(0),
        rollout_(*systemDynamicsPtr, piSettings_.rolloutSettings_) {
    // TODO(jcarius) how to ensure that we are given a control affine system?
    // TODO(jcarius) how to ensure that we are given a suitable cost function?
    // TODO(jcarius) how to ensure that the constraint is input-affine and full row-rank D?

    if (piSettings_.rolloutSettings_.integratorType_ != IntegratorType::EULER) {
      throw std::runtime_error("PiSolver only works with Euler Integration.");
    }

    auto seed = static_cast<unsigned int>(time(nullptr));
    std::cerr << "Setting random seed to controller: " << seed << std::endl;
    controller_.setRandomSeed(seed);
  }

  ~PiSolver() override = default;

  void reset() override {
    this->costDesiredTrajectories_.clear();
    this->costDesiredTrajectoriesBuffer_.clear();
    nominalTimeTrajectoriesStock_.clear();
    nominalStateTrajectoriesStock_.clear();
    nominalInputTrajectoriesStock_.clear();
    nominalControllersStock_.clear();
    numIterations_ = 0;
  }

  void run(scalar_t initTime, const state_vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes) override {
    numIterations_++;

    this->updateCostDesiredTrajectories();
    costFunction_->setCostDesiredTrajectories(this->costDesiredTrajectories_);

    const auto numSteps = static_cast<size_t>(std::round((finalTime - initTime) / piSettings_.rolloutSettings_.minTimeStep_)) + 1;

    // setup containers to store rollout data
    state_vector_array2_t state_vector_array2(piSettings_.numSamples_, state_vector_array_t(numSteps));      // vector of vectors of states
    input_vector_array2_t noisyInputVector_array2(piSettings_.numSamples_, input_vector_array_t(numSteps));  // vector of vectors of inputs
    scalar_array2_t stageCost(piSettings_.numSamples_, scalar_array_t(numSteps, 0.0));                       // vector of vectors of costs

    controller_.cacheResults_ = true;
    controller_.cacheData_.reserve(numSteps + 2);  // TODO(jcarius) check if this is the size at the end

    // -------------------------------------------------------------------------
    // forward rollout
    // -------------------------------------------------------------------------

    // a sample is a single stochastic rollout from initTime to finalTime
    for (size_t sample = 0; sample < piSettings_.numSamples_; sample++) {
      controller_.cacheData_.clear();

      typename rollout_t::state_vector_array_t stateTrajectory;
      typename rollout_t::scalar_array_t timeTrajectory;
      {
        // braces to guard against usage of temporary rollout quantities
        typename rollout_t::size_array_t eventsPastTheEndIndeces;
        typename rollout_t::input_vector_array_t inputTrajectory;
        rollout_.run(0, initTime, initState, finalTime, &controller_, *Base::getLogicRulesMachinePtr(), timeTrajectory,
                     eventsPastTheEndIndeces, stateTrajectory, inputTrajectory);
        // see Euler-Maruyama method (https://infoscience.epfl.ch/record/143450/files/sde_tutorial.pdf)
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

      // interpret NaN in cost as infinity
      std::for_each(stageCost[sample].begin(), stageCost[sample].end(),
                    [](scalar_t& in) { in = std::isnan(in) ? std::numeric_limits<scalar_t>::infinity() : in; });
    }

    // -------------------------------------------------------------------------
    // backward pass
    // collect cost to go and calculate psi and input across all samples
    // -------------------------------------------------------------------------
    scalar_array2_t J(piSettings_.numSamples_,
                      scalar_array_t(numSteps, 0.0));  // value of J (cost-to-go) for each sample and each time step
    scalar_array_t psiDistorted(numSteps, 0.0);  // value of Psi for each time step, averaged over samples (distortion = scaling of exp
                                                 // argument for numerics, no division by numSamples)
    input_vector_array_t u_opt(numSteps, input_vector_t::Zero());  // value of optimal input across time steps, averaged over samples

    // initialize J for each sample and find min across samples
    scalar_t minJ_currStep = std::numeric_limits<scalar_t>::max();
    scalar_t maxJ_currStep = 0;
    for (size_t sample = 0; sample < piSettings_.numSamples_; sample++) {
      J[sample][numSteps - 1] = stageCost[sample][numSteps - 1];

      if (J[sample][numSteps - 1] < minJ_currStep) {
        minJ_currStep = J[sample][numSteps - 1];
      }
      if (std::isfinite(J[sample][numSteps - 1]) && J[sample][numSteps - 1] > maxJ_currStep) {
        maxJ_currStep = J[sample][numSteps - 1];
      }
    }

    if (maxJ_currStep - minJ_currStep < 1.0) {
      // the purpose of maxJ is to smooth the softmax
      maxJ_currStep = minJ_currStep + 1.0;
    }

    // initialize psi
    psiDistorted[numSteps - 1] = std::accumulate(
        J.begin(), J.end(), scalar_t(0.0), [this, numSteps, minJ_currStep, maxJ_currStep](scalar_t a, const scalar_array_t& Ji) {
          return std::move(a) + std::exp(-(Ji[numSteps - 1] - minJ_currStep) / (maxJ_currStep - minJ_currStep) / piSettings_.gamma_);
        });

    // initialize u for each sample
    for (size_t sample = 0; sample < piSettings_.numSamples_; sample++) {
      u_opt[numSteps - 1] += noisyInputVector_array2[sample][numSteps - 1] *
                             std::exp(-(J[sample][numSteps - 1] - minJ_currStep) / (maxJ_currStep - minJ_currStep) / piSettings_.gamma_);
    }
    u_opt[numSteps - 1] /= psiDistorted[numSteps - 1];

    // propagate towards initial time
    for (int n = numSteps - 2; n >= 0; n--) {
      // calculate cost-to-go for this step for each sample
      scalar_t minJ_currStep = std::numeric_limits<scalar_t>::max();
      scalar_t maxJ_currStep = 0;
      for (size_t sample = 0; sample < piSettings_.numSamples_; sample++) {
        J[sample][n] = J[sample][n + 1] + stageCost[sample][n];

        if (J[sample][n] < minJ_currStep) {
          minJ_currStep = J[sample][n];
        }
        if (std::isfinite(J[sample][n]) && J[sample][n] > maxJ_currStep) {
          maxJ_currStep = J[sample][n];
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

      psiDistorted[n] =
          std::accumulate(J.begin(), J.end(), scalar_t(0.0), [this, n, minJ_currStep, maxJ_currStep](scalar_t a, const scalar_array_t& Ji) {
            return std::move(a) + std::exp(-(Ji[n] - minJ_currStep) / (maxJ_currStep - minJ_currStep) / piSettings_.gamma_);
          });

      if (psiDistorted[n] / piSettings_.numSamples_ < 0.01) {
        std::cerr << "Warning: Less than ~1% of samples are significant in step " << n << std::endl;
      }

      // u_opt
      for (size_t sample = 0; sample < piSettings_.numSamples_; sample++) {
        u_opt[n] += noisyInputVector_array2[sample][n] *
                    std::exp(-(J[sample][n] - minJ_currStep) / (maxJ_currStep - minJ_currStep) / piSettings_.gamma_);
      }
      u_opt[n] /= psiDistorted[n];
    }

    // -------------------------------------------------------------------------
    // save data
    // -------------------------------------------------------------------------

    // time trajectory
    nominalTimeTrajectoriesStock_.clear();
    nominalTimeTrajectoriesStock_.push_back(scalar_array_t(numSteps));
    std::generate(nominalTimeTrajectoriesStock_[0].begin(), nominalTimeTrajectoriesStock_[0].end(),
                  [n = 0, initTime, this]() mutable { return initTime + (n++) * piSettings_.rolloutSettings_.minTimeStep_; });

    // input trajectory
    nominalInputTrajectoriesStock_.clear();
    nominalInputTrajectoriesStock_.push_back(u_opt);

    // state trajectory: perform rollout without noise but with input trajectory.
    // This also sets sampling policy for next iteration
    std::unique_ptr<ControllerBase<STATE_DIM, INPUT_DIM>> ffwCtrl(
        new FeedforwardController<STATE_DIM, INPUT_DIM>(nominalTimeTrajectoriesStock_[0], nominalInputTrajectoriesStock_[0]));
    controller_.setSamplingPolicy(std::move(ffwCtrl));
    controller_.gamma_ = 0.0;
    controller_.cacheResults_ = false;

    typename rollout_t::scalar_array_t timeTrajectoryNominal;
    typename rollout_t::size_array_t eventsPastTheEndIndecesNominal;
    typename rollout_t::state_vector_array_t stateTrajectoryNominal;
    typename rollout_t::input_vector_array_t inputTrajectoryNominal;
    rollout_.run(0, initTime, initState, finalTime, &controller_, *Base::getLogicRulesMachinePtr(), timeTrajectoryNominal,
                 eventsPastTheEndIndecesNominal, stateTrajectoryNominal, inputTrajectoryNominal);

    nominalStateTrajectoriesStock_.clear();
    nominalStateTrajectoriesStock_.push_back(stateTrajectoryNominal);

    // controller for ROS transmission
    nominalControllersStock_.clear();
    nominalControllersStock_.emplace_back(pi_controller_t(controller_));

    // prepare local controller for next iteration
    controller_.gamma_ = piSettings_.gamma_;

    // debug printing
    if (piSettings_.debugPrint_ > 0) {
      std::cerr << "\e[1m++++++++++++++++++++ Debug Print Iteration " << numIterations_ << "++++++++++++++++++++\e[0m" << std::endl;
      std::cerr << "mpc init state: " << initState.transpose() << std::endl;

      std::cerr << "After softmax: Setting u_opt[0] = " << nominalInputTrajectoriesStock_[0][0] << std::endl;

      if (piSettings_.debugPrint_ > 1) {
        std::cerr << "\n------- Computation of the optimal control at current time -----" << std::endl;
        for (size_t sample = 0; sample < piSettings_.numSamples_; sample++) {
          std::cerr << "sample " << sample << " initNoisyInput " << noisyInputVector_array2[sample][0].transpose() << " init cost-to-go "
                    << J[sample][0] << std::endl;
        }

        std::cerr << "\n------- The optimized PiController ----------------" << std::endl;
        controller_.display();

        std::cerr << "\n------- The predicted state trajectory ----------------" << std::endl;
        for (const auto& state_i : stateTrajectoryNominal) {
          std::cerr << state_i.transpose() << std::endl;
        }
      }

      if (piSettings_.debugPrint_ > 2) {
        printIterationDebug(initTime, state_vector_array2, noisyInputVector_array2, J);
      }

      std::cerr << "Finished Debug Print Iteration " << numIterations_ << std::endl;
    }
  }

  void run(scalar_t initTime, const state_vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes,
           const controller_ptr_array_t& controllersStock) override {
    throw std::runtime_error("not implemented.");
  }

  void blockwiseMovingHorizon(bool flag) override {
    if (flag) {
      std::cerr << "[PiSolver] BlockwiseMovingHorizon enabled." << std::endl;
    }
  }

  void getPerformanceIndeces(scalar_t& costFunction, scalar_t& constraint1ISE, scalar_t& constraint2ISE) const override {
    throw std::runtime_error("not implemented.");
  }

  size_t getNumIterations() const override { return numIterations_; }

  void getIterationsLog(eigen_scalar_array_t& iterationCost, eigen_scalar_array_t& iterationISE1,
                        eigen_scalar_array_t& iterationISE2) const override {
    throw std::runtime_error("not implemented.");
  }

  void getIterationsLogPtr(const eigen_scalar_array_t*& iterationCostPtr, const eigen_scalar_array_t*& iterationISE1Ptr,
                           const eigen_scalar_array_t*& iterationISE2Ptr) const override {
    throw std::runtime_error("not implemented.");
  }

  scalar_t getFinalTime() const final { throw std::runtime_error("not implemented."); }

  const scalar_array_t& getPartitioningTimes() const final { throw std::runtime_error("not implemented."); }

  /**
   * @brief Sets the initial sampling policy for path integral rollouts
   * @param The control policy to sample around
   */
  void setSamplingPolicy(std::unique_ptr<ControllerBase<STATE_DIM, INPUT_DIM>> samplingPolicy) {
    controller_.setSamplingPolicy(std::move(samplingPolicy));
  }

  void getPrimalSolutionPtr(scalar_t finalTime, primal_solution_t* primalSolutionPtr) const final {
    // total number of nodes
    int N = 0;
    for (const scalar_array_t& timeTrajectory_i : nominalTimeTrajectoriesStock_) {
      N += timeTrajectory_i.size();
    }

    // fill trajectories
    primalSolutionPtr->timeTrajectory_.clear();
    primalSolutionPtr->timeTrajectory_.reserve(N);
    primalSolutionPtr->stateTrajectory_.clear();
    primalSolutionPtr->stateTrajectory_.reserve(N);
    primalSolutionPtr->inputTrajectory_.clear();
    primalSolutionPtr->inputTrajectory_.reserve(N);
    for (int i = 0; i < nominalTimeTrajectoriesStock_.size(); i++) {
      primalSolutionPtr->timeTrajectory_.insert(primalSolutionPtr->timeTrajectory_.end(), nominalTimeTrajectoriesStock_[i].begin(),
                                                nominalTimeTrajectoriesStock_[i].end());
      primalSolutionPtr->stateTrajectory_.insert(primalSolutionPtr->stateTrajectory_.end(), nominalStateTrajectoriesStock_[i].begin(),
                                                 nominalStateTrajectoriesStock_[i].end());
      primalSolutionPtr->inputTrajectory_.insert(primalSolutionPtr->inputTrajectory_.end(), nominalInputTrajectoriesStock_[i].begin(),
                                                 nominalInputTrajectoriesStock_[i].end());
    }

    // fill controller
    if (piSettings_.useFeedbackPolicy_) {
      primalSolutionPtr->controllerPtr_.reset();
      // concatenate controller stock into a single controller
      for (const pi_controller_t& controller_i : nominalControllersStock_) {
        if (controller_i.empty()) {
          continue;  // some time partitions may be unused
        }

        if (primalSolutionPtr->controllerPtr_) {
          primalSolutionPtr->controllerPtr_->concatenate(&controller_i);
        } else {
          primalSolutionPtr->controllerPtr_.reset(controller_i.clone());
        }
      }
    } else {
      primalSolutionPtr->controllerPtr_.reset(
          new feedforward_controller_t(primalSolutionPtr->timeTrajectory_, primalSolutionPtr->inputTrajectory_));
    }

    // fill logic
    primalSolutionPtr->eventTimes_ = this->getLogicRulesPtr()->eventTimes();
    primalSolutionPtr->subsystemsSequence_ = this->getLogicRulesPtr()->subsystemsSequence();
  }

  void rewindOptimizer(size_t firstIndex) override {}

  const unsigned long long int& getRewindCounter() const override { throw std::runtime_error("not implemented."); }

  void printIterationDebug(scalar_t initTime, const state_vector_array2_t& state_vector_array2,
                           const input_vector_array2_t& input_vector_array2, const scalar_array2_t& J) {
    if (input_vector_array2.size() != state_vector_array2.size() || state_vector_array2.size() != J.size()) {
      throw std::runtime_error("printIterationDebug: Number of samples do not match");
    }

    // proper spacing and formatting
    constexpr int timeWidth = 11;
    constexpr int jWidth = 12;
    constexpr int stateWidth = 11 * STATE_DIM;
    constexpr int inputWidth = 11 * INPUT_DIM;

    const auto defaultPrecision = std::cerr.precision();
    std::cerr << std::setprecision(3);
    std::cerr << std::scientific;
    Eigen::IOFormat eigenFormat(Eigen::StreamPrecision, Eigen::DontAlignCols);

    for (int sample = 0; sample < J.size(); sample++) {
      std::cerr << "+++ Sample # " << sample << " +++\n";
      std::cerr << std::setw(timeWidth) << "time" << std::setw(jWidth) << "cost-to-go" << std::setw(stateWidth) << "state"
                << std::setw(inputWidth) << "input" << std::endl;
      for (int n = 0; n < J[sample].size(); n++) {
        std::cerr << std::setw(timeWidth) << initTime + n * piSettings_.rolloutSettings_.minTimeStep_ << std::setw(jWidth) << J[sample][n]
                  << std::setw(stateWidth) << state_vector_array2[sample][n].transpose().format(eigenFormat) << std::setw(inputWidth)
                  << input_vector_array2[sample][n].transpose().format(eigenFormat) << std::endl;
      }
    }

    std::cerr << std::defaultfloat;  // revert forced scientific notation
    std::cerr << std::setprecision(defaultPrecision);
  }

  scalar_t getValueFunction(scalar_t time, const state_vector_t& state) const override { throw std::runtime_error("Not implemented."); }

  void getValueFunctionStateDerivative(scalar_t time, const state_vector_t& state, state_vector_t& Vx) const override {
    throw std::runtime_error("Not implemented.");
  }

  void getStateInputConstraintLagrangian(scalar_t time, const state_vector_t& state, dynamic_vector_t& nu) const override {
    throw std::runtime_error("Not implemented.");
  }

 protected:
  PI_Settings piSettings_;  //! path integral settings

  typename controlled_system_base_t::Ptr systemDynamics_;

  std::unique_ptr<cost_function_t> costFunction_;  //! cost function owned by solver
  constraint_t constraint_;                        //! constraint owned by solver

  pi_controller_t controller_;  //! internal controller used for rollouts

  size_t numIterations_;

  rollout_t rollout_;

  std::vector<scalar_array_t> nominalTimeTrajectoriesStock_;
  state_vector_array2_t nominalStateTrajectoriesStock_;
  input_vector_array2_t nominalInputTrajectoriesStock_;
  std::vector<pi_controller_t> nominalControllersStock_;
};

}  // namespace ocs2
