#pragma once

#include <ocs2_oc/rollout/RolloutBase.h>
#include <raisim/World.hpp>

namespace ocs2 {

/**
 * This rollout class uses the Raisim physics simulator for integrating the system dynamics
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class RaisimRollout : public RolloutBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = RolloutBase<STATE_DIM, INPUT_DIM>;

  using controller_t = typename Base::controller_t;
  using size_array_t = typename Base::size_array_t;
  using scalar_t = typename Base::scalar_t;
  using scalar_array_t = typename Base::scalar_array_t;
  using state_vector_t = typename Base::state_vector_t;
  using state_vector_array_t = typename Base::state_vector_array_t;
  using input_vector_t = typename Base::input_vector_t;
  using input_vector_array_t = typename Base::input_vector_array_t;

  using logic_rules_machine_t = HybridLogicRulesMachine;

  explicit RaisimRollout(std::string pathToUrdf, Rollout_Settings rolloutSettings = Rollout_Settings(), char algorithmName[] = nullptr)
      : Base(std::move(rolloutSettings), std::move(algorithmName)) {
    system_ = world_.addArticulatedSystem(pathToUrdf);
    ground_ = world_.addGround();
    world_.setTimeStep(this->settings().minTimeStep_);
  }

  state_vector_t run(size_t partitionIndex, scalar_t initTime, const state_vector_t& initState, scalar_t finalTime,
                     controller_t* controller, logic_rules_machine_t& logicRulesMachine, scalar_array_t& timeTrajectory,
                     size_array_t& eventsPastTheEndIndeces, state_vector_array_t& stateTrajectory,
                     input_vector_array_t& inputTrajectory) override {
    std::cout << "RaisimRollout::run called at time " << initTime << std::endl;

    // Prepare arrays
    const auto numSteps = static_cast<int>(std::round((finalTime - initTime) / this->settings().minTimeStep_));
    timeTrajectory.clear();
    timeTrajectory.reserve(numSteps + 1);
    stateTrajectory.clear();
    stateTrajectory.reserve(numSteps + 1);
    inputTrajectory.clear();
    inputTrajectory.reserve(numSteps + 1);

    // Set inital state to simulation
    //! todo: use general conversion function passed in by the user
    std::cout << "RAI dof = " << system_->getDOF() << std::endl;
    system_->setState(initState.template head<2>(), initState.template tail<2>());

    // Forward simulate
    for (int i = 0; i < numSteps; i++) {
      world_.integrate1();  //! prepares all dynamical quantities for current time step

      //! @todo extract required quantities here
      Eigen::MatrixXd M = system_->getMassMatrix().e();
      std::cout << "M\n" << M << std::endl;

      Eigen::VectorXd raisim_q, raisim_dq;
      system_->getState(raisim_q, raisim_dq);

      const auto time = initTime + i * this->settings().minTimeStep_;
      timeTrajectory.push_back(time);
      stateTrajectory.emplace_back(state_vector_t());
      stateTrajectory.back() << raisim_q, raisim_dq;
      std::cout << "state: " << stateTrajectory.back().transpose() << std::endl;
      input_vector_t input = controller->computeInput(time, stateTrajectory.back());
      std::cout << "input: " << input.transpose() << std::endl;
      inputTrajectory.emplace_back(input);

      Eigen::VectorXd tau(2);
      tau.setZero();
      tau.head(1) = inputTrajectory.back();
      system_->setGeneralizedForce(tau);

      world_.integrate2();
    }
    assert(initState.isApprox(stateTrajectory.front()));

    // also push back final state and input
    timeTrajectory.push_back(initTime + numSteps * this->settings().minTimeStep_);
    Eigen::VectorXd raisim_q, raisim_dq;
    system_->getState(raisim_q, raisim_dq);
    stateTrajectory.emplace_back(state_vector_t());
    stateTrajectory.back() << raisim_q, raisim_dq;
    std::cout << "state: " << stateTrajectory.back().transpose() << std::endl;
    input_vector_t input = controller->computeInput(timeTrajectory.back(), stateTrajectory.back());
    std::cout << "input: " << input.transpose() << std::endl;
    inputTrajectory.emplace_back(input);
  }

 protected:
  raisim::World world_;
  raisim::Ground* ground_;
  raisim::ArticulatedSystem* system_;
};

}  // namespace ocs2
