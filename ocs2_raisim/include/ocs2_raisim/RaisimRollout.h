/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#pragma once

#include <ocs2_oc/rollout/RolloutBase.h>
#include <raisim/World.hpp>

#ifdef USE_RAISIM_VISUALIZER
#include <raisim/OgreVis.hpp>
#endif

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

  using typename Base::controller_t;
  using typename Base::input_vector_array_t;
  using typename Base::input_vector_t;
  using typename Base::scalar_array_t;
  using typename Base::scalar_t;
  using typename Base::size_array_t;
  using typename Base::state_vector_array_t;
  using typename Base::state_vector_t;

  using logic_rules_machine_t = HybridLogicRulesMachine;

  using state_to_raisim_gen_coord_gen_vel_t =
      std::function<std::pair<Eigen::VectorXd, Eigen::VectorXd>(const state_vector_t&, const input_vector_t&)>;
  using raisim_gen_coord_gen_vel_to_state_t = std::function<state_vector_t(const Eigen::VectorXd&, const Eigen::VectorXd&)>;
  using input_to_raisim_generalized_force_t =
      std::function<Eigen::VectorXd(double, const input_vector_t&, const state_vector_t&, const Eigen::VectorXd&, const Eigen::VectorXd&)>;
  using data_extraction_callback_t = std::function<void(double, const raisim::ArticulatedSystem&)>;

  /**
   * @brief Constructor
   * @param[in] pathToUrdf: Full file path to the urdf description for initializing the simulator
   * @param[in] stateToRaisimGenCoordGenVel: Transformation function that converts ocs2 state to generalized coordinate and generalized
   * velocity used by Raisim
   * @param[in] raisimGenCoordGenVelToState: Transformation function that converts Raisim generalized coordinates and velocities to ocs2
   * state
   * @param[in] inputToRaisimGeneralizedForce: Tranformation function that converts ocs2 control input to Raisim generalized force
   * @param[in] orderedJointNames: Ordered vector of joint names. Parents must be named before children, names must be identical to URDF
   * joints
   * @param[in] dataExtractionCallback: Optional callback function to extract user-defined information from the simulation at each timestep
   * @param[in] rolloutSettings
   * @param[in] algorithmName
   */
  RaisimRollout(const std::string& pathToUrdf, state_to_raisim_gen_coord_gen_vel_t stateToRaisimGenCoordGenVel,
                raisim_gen_coord_gen_vel_to_state_t raisimGenCoordGenVelToState,
                input_to_raisim_generalized_force_t inputToRaisimGeneralizedForce, const std::vector<std::string>& orderedJointNames = {},
                data_extraction_callback_t dataExtractionCallback = nullptr, Rollout_Settings rolloutSettings = Rollout_Settings(),
                char algorithmName[] = nullptr)
      : Base(std::move(rolloutSettings), std::move(algorithmName)),
        setSimulatorStateOnRolloutRunAlways_(true),
        setSimulatorStateOnRolloutRunOnce_(false),
        stateToRaisimGenCoordGenVel_(std::move(stateToRaisimGenCoordGenVel)),
        raisimGenCoordGenVelToState_(std::move(raisimGenCoordGenVelToState)),
        inputToRaisimGeneralizedForce_(std::move(inputToRaisimGeneralizedForce)),
        dataExtractionCallback_(std::move(dataExtractionCallback)) {
    world_.setTimeStep(this->settings().minTimeStep_);
    system_ = world_.addArticulatedSystem(pathToUrdf, "", orderedJointNames);

    std::cerr << "\nInstatiated Raisim System with DoF = " << system_->getDOF() << std::endl;
    const auto bodyNames = system_->getBodyNames();
    std::cerr << "Body Names are";
    for (const auto& bodyName : bodyNames) {
      std::cerr << "\t" << bodyName;
    }
    std::cerr << std::endl;

    ground_ = world_.addGround();
    //    world_.setERP(0, 0);

#ifdef USE_RAISIM_VISUALIZER
    auto vis = raisim::OgreVis::get();
    vis->setWorld(&world_);
    vis->setWindowSize(1280, 720);
    // more setup options here: https://github.com/leggedrobotics/raisimGym/blob/master/raisim_gym/env/env/ANYmal/Environment.hpp
    vis->initApp();

    systemVisual_ = vis->createGraphicalObject(system_, "system");
    groundVisual_ = vis->createGraphicalObject(ground_, 20, "floor", "checkerboard_green");

    vis->setDesiredFPS(30);
    vis->select(groundVisual_->at(0), false);
    vis->getCameraMan()->setYawPitchDist(Ogre::Radian(M_PI_4), Ogre::Radian(-1.3), 3, true);
#endif
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

    // Set inital state to simulation if requested
    if (setSimulatorStateOnRolloutRunAlways_ or setSimulatorStateOnRolloutRunOnce_) {
      Eigen::VectorXd q_init, dq_init;
      std::tie(q_init, dq_init) = stateToRaisimGenCoordGenVel_(initState, controller->computeInput(initTime, initState));
      assert(system_->getGeneralizedCoordinateDim() == q_init.rows());
      system_->setState(q_init, dq_init);
      setSimulatorStateOnRolloutRunOnce_ = false;
    }

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
      inputTrajectory.push_back(input);

      Eigen::VectorXd tau = inputToRaisimGeneralizedForce_(time, input, stateTrajectory.back(), raisim_q, raisim_dq);
      assert(tau.rows() == system_->getDOF());
      system_->setGeneralizedForce(tau);

      world_.integrate2();

#ifdef USE_RAISIM_VISUALIZER
      auto vis = raisim::OgreVis::get();
      vis->renderOneFrame();
#endif
    }

    if (setSimulatorStateOnRolloutRunAlways_) {
      assert(initState.isApprox(stateTrajectory.front()));
    }

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
    inputTrajectory.push_back(input);

    return stateTrajectory.back();
  }

 public:
  bool setSimulatorStateOnRolloutRunAlways_;  //! Whether or not to always set the starting state of the rollout to the simulator
  bool setSimulatorStateOnRolloutRunOnce_;    //! Whether or not to set the starting state to the simulator at the next rollout call only

 protected:
  // Handles to Raisim objects
  raisim::World world_;
  raisim::Ground* ground_;
  raisim::ArticulatedSystem* system_;

#ifdef USE_RAISIM_VISUALIZER
  // Handles to Raisim visualization objects
  std::vector<raisim::GraphicObject>* systemVisual_;
  std::vector<raisim::GraphicObject>* groundVisual_;
#endif

  // Robot-specific conversion function handles
  state_to_raisim_gen_coord_gen_vel_t stateToRaisimGenCoordGenVel_;
  raisim_gen_coord_gen_vel_to_state_t raisimGenCoordGenVelToState_;
  input_to_raisim_generalized_force_t inputToRaisimGeneralizedForce_;
  data_extraction_callback_t dataExtractionCallback_;
};

}  // namespace ocs2
