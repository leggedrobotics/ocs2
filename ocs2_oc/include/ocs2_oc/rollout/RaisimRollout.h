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
class RaisimRollout final : public RolloutBase<STATE_DIM, INPUT_DIM> {
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

  using state_to_raisim_gen_coord_gen_vel_t = std::function<std::pair<Eigen::VectorXd, Eigen::VectorXd>(state_vector_t)>;
  using raisim_gen_coord_gen_vel_to_state_t = std::function<state_vector_t(const Eigen::VectorXd&, const Eigen::VectorXd&)>;
  using input_to_raisim_generalized_force_t = std::function<Eigen::VectorXd(const input_vector_t&, const state_vector_t&)>;
  using data_extraction_callback_t = std::function<void(double, const raisim::ArticulatedSystem&)>;

  /**
   * @brief Constructor
   * @param[in] pathToUrdf: Full file path to the urdf description for initializing the simulator
   * @param[in] stateToRaisimGenCoordGenVel: Transformation function that converts ocs2 state to generalized coordinate and generalized
   * velocity used by RAIsim
   * @param[in] raisimGenCoordGenVelToState: Transformation function that converts RAIsim generalized coordinates and velocities to ocs2
   * state
   * @param[in] inputToRaisimGeneralizedForce: Tranformation function that converts ocs2 control input to RAIsim generalized force
   * @param[in] dataExtractionCallback: Optional callback function to extract user-defined information from the simulation at each timestep
   * @param[in] rolloutSettings
   * @param[in] algorithmName
   */
  RaisimRollout(const std::string& pathToUrdf, state_to_raisim_gen_coord_gen_vel_t stateToRaisimGenCoordGenVel,
                raisim_gen_coord_gen_vel_to_state_t raisimGenCoordGenVelToState,
                input_to_raisim_generalized_force_t inputToRaisimGeneralizedForce,
                data_extraction_callback_t dataExtractionCallback = nullptr, Rollout_Settings rolloutSettings = Rollout_Settings(),
                char algorithmName[] = nullptr)
      : Base(std::move(rolloutSettings), std::move(algorithmName)),
        stateToRaisimGenCoordGenVel_(std::move(stateToRaisimGenCoordGenVel)),
        raisimGenCoordGenVelToState_(std::move(raisimGenCoordGenVelToState)),
        inputToRaisimGeneralizedForce_(std::move(inputToRaisimGeneralizedForce)),
        dataExtractionCallback_(dataExtractionCallback) {
    system_ = world_.addArticulatedSystem(pathToUrdf);
    ground_ = world_.addGround();
    world_.setTimeStep(this->settings().minTimeStep_);
  }

  state_vector_t run(size_t partitionIndex, scalar_t initTime, const state_vector_t& initState, scalar_t finalTime,
                     controller_t* controller, logic_rules_machine_t& logicRulesMachine, scalar_array_t& timeTrajectory,
                     size_array_t& eventsPastTheEndIndeces, state_vector_array_t& stateTrajectory,
                     input_vector_array_t& inputTrajectory) override {
    // Prepare arrays
    const auto numSteps = static_cast<int>(std::round((finalTime - initTime) / this->settings().minTimeStep_));
    timeTrajectory.clear();
    timeTrajectory.reserve(numSteps + 1);
    stateTrajectory.clear();
    stateTrajectory.reserve(numSteps + 1);
    inputTrajectory.clear();
    inputTrajectory.reserve(numSteps + 1);

    // Set inital state to simulation
    Eigen::VectorXd q_init, dq_init;
    std::tie(q_init, dq_init) = stateToRaisimGenCoordGenVel_(initState);
    system_->setState(q_init, dq_init);

    // Forward simulate
    for (int i = 0; i < numSteps; i++) {
      world_.integrate1();  // prepares all dynamical quantities for current time step

      Eigen::VectorXd raisim_q, raisim_dq;
      system_->getState(raisim_q, raisim_dq);

      const auto time = initTime + i * this->settings().minTimeStep_;
      timeTrajectory.push_back(time);

      if (dataExtractionCallback_) {
        dataExtractionCallback_(time, *system_);
      }

      stateTrajectory.emplace_back(raisimGenCoordGenVelToState_(raisim_q, raisim_dq));

      input_vector_t input = controller->computeInput(time, stateTrajectory.back());
      inputTrajectory.emplace_back(input);
      system_->setGeneralizedForce(inputToRaisimGeneralizedForce_(input, stateTrajectory.back()));

      world_.integrate2();
    }
    assert(initState.isApprox(stateTrajectory.front()));

    // also push back final state and input
    timeTrajectory.push_back(initTime + numSteps * this->settings().minTimeStep_);

    world_.integrate1();

    Eigen::VectorXd raisim_q, raisim_dq;
    system_->getState(raisim_q, raisim_dq);
    stateTrajectory.emplace_back(raisimGenCoordGenVelToState_(raisim_q, raisim_dq));

    if (dataExtractionCallback_) {
      dataExtractionCallback_(timeTrajectory.back(), *system_);
    }

    input_vector_t input = controller->computeInput(timeTrajectory.back(), stateTrajectory.back());
    inputTrajectory.emplace_back(input);
  }

 protected:
  // Handles to RAIsim objects
  raisim::World world_;
  raisim::Ground* ground_;
  raisim::ArticulatedSystem* system_;

  // Robot-specific conversions
  state_to_raisim_gen_coord_gen_vel_t stateToRaisimGenCoordGenVel_;
  raisim_gen_coord_gen_vel_to_state_t raisimGenCoordGenVelToState_;
  input_to_raisim_generalized_force_t inputToRaisimGeneralizedForce_;

  data_extraction_callback_t dataExtractionCallback_;
};

}  // namespace ocs2
