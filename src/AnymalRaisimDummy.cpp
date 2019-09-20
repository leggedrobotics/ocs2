#include <ros/package.h>

#include <ocs2_anymal_interface/OCS2AnymalInterface.h>
#include <ocs2_oc/rollout/RaisimRollout.h>
#include <ocs2_quadruped_interface/test/MRT_ROS_Dummy_Quadruped.h>
#include <ocs2_anymal_interface/AnymalRaisimConversions.hpp>

int main(int argc, char* argv[]) {
  // task file
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFileFolderName = std::string(argv[1]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

  constexpr int STATE_DIM = 24;
  constexpr int INPUT_DIM = 24;
  constexpr int JOINT_COORD_SIZE = 12;

  std::shared_ptr<anymal::OCS2AnymalInterface> anymal_interface(
      new anymal::OCS2AnymalInterface(ros::package::getPath("ocs2_anymal_interface") + "/config/" + taskFileFolderName));

  // setup MRT with simulator rollouts
  using mrt_t = switched_model::MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>;
  std::shared_ptr<mrt_t> anymal_mrt(new mrt_t(anymal_interface, "anymal"));

  anymal::AnymalRaisimConversions conversions(anymal_mrt, anymal_interface);
  {
    std::vector<std::string> orderedJointNames{"LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE",
                                               "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
    using sim_rollout_t = ocs2::RaisimRollout<STATE_DIM, INPUT_DIM>;
    std::unique_ptr<sim_rollout_t> simRollout(new sim_rollout_t(
        ros::package::getPath("ocs2_anymal_interface") + "/urdf/anymal.urdf",
        std::bind(&anymal::AnymalRaisimConversions::stateToRaisimGenCoordGenVel, &conversions, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&anymal::AnymalRaisimConversions::raisimGenCoordGenVelToState, &conversions, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&anymal::AnymalRaisimConversions::inputToRaisimGeneralizedForce, &conversions, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
        orderedJointNames,
        std::bind(&anymal::AnymalRaisimConversions::extractModelData, &conversions, std::placeholders::_1, std::placeholders::_2)));

    simRollout->setSimulatorStateOnRolloutRunAlways_ = false;
    simRollout->setSimulatorStateOnRolloutRunOnce_ = true;

    anymal_mrt->initRollout(std::move(simRollout));
  }

  switched_model::MRT_ROS_Dummy_Quadruped<JOINT_COORD_SIZE> mrt_dummy_loop(anymal_interface, anymal_mrt,
                                                                           anymal_interface->mpcSettings().mrtDesiredFrequency_, "anymal",
                                                                           anymal_interface->mpcSettings().mpcDesiredFrequency_);

  mrt_dummy_loop.launchNodes(argc, argv);

  // initial state
  anymal::OCS2AnymalInterface::rbd_state_vector_t initRbdState;
  anymal_interface->getLoadedInitialState(initRbdState);
  mrt_t::system_observation_t initObservation;
  initObservation.time() = 0.0;
  anymal_interface->computeSwitchedModelState(initRbdState, initObservation.state());
  initObservation.input().setZero();
  initObservation.subsystem() = 15;

  // initial command
  mrt_t::cost_desired_trajectories_t initCostDesiredTrajectories;

  // time
  auto& timeTrajectory = initCostDesiredTrajectories.desiredTimeTrajectory();
  timeTrajectory.resize(1);
  timeTrajectory[0] = 0.0;

  // State
  auto& stateTrajectory = initCostDesiredTrajectories.desiredStateTrajectory();
  stateTrajectory.resize(1);
  stateTrajectory[0] = initObservation.state();

  // Input
  auto& inputTrajectory = initCostDesiredTrajectories.desiredInputTrajectory();
  inputTrajectory.resize(1);
  inputTrajectory[0] = initObservation.input();

  mrt_dummy_loop.run(initObservation, initCostDesiredTrajectories);

  return 0;
}
