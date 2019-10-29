#include <ros/package.h>

#include <ocs2_anymal_bear/AnymalBearInterface.h>
#include <ocs2_anymal_raisim/AnymalRaisimConversions.h>
#include <ocs2_quadruped_interface/test/MRT_ROS_Dummy_Quadruped.h>
#include <ocs2_raisim/RaisimRollout.h>

int main(int argc, char* argv[]) {
  // task file
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFileFolderName = std::string(argv[1]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

  ros::init(argc, argv, "anymal_visualizer");

  constexpr int STATE_DIM = 24;
  constexpr int INPUT_DIM = 24;
  constexpr int JOINT_COORD_SIZE = 12;

  using interface_t = anymal::AnymalBearInterface;
  std::shared_ptr<interface_t> anymal_interface(
      new interface_t(ros::package::getPath("ocs2_anymal_bear") + "/config/" + taskFileFolderName));

  std::string urdf_param = "/ocs2_anymal_bear_description";

  std::string urdf;
  if (!ros::param::get(urdf_param, urdf)) {
    throw ros::Exception("Error reading ros parameter: " + urdf_param);
  }

  // setup MRT with simulator rollouts
  using mrt_t = switched_model::MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>;
  std::shared_ptr<mrt_t> anymal_mrt(new mrt_t(anymal_interface, "anymal"));

  anymal::AnymalRaisimConversions conversions;

  std::vector<std::string> orderedJointNames{"LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE",
                                             "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
  ocs2::RaisimRollout<STATE_DIM, INPUT_DIM> simRollout(
      urdf,
      std::bind(&anymal::AnymalRaisimConversions::stateToRaisimGenCoordGenVel, &conversions, std::placeholders::_1, std::placeholders::_2),
      std::bind(&anymal::AnymalRaisimConversions::raisimGenCoordGenVelToState, &conversions, std::placeholders::_1, std::placeholders::_2),
      std::bind(&anymal::AnymalRaisimConversions::inputToRaisimGeneralizedForce, &conversions, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
      orderedJointNames,
      std::bind(&anymal::AnymalRaisimConversions::extractModelData, &conversions, std::placeholders::_1, std::placeholders::_2));

  simRollout.setSimulatorStateOnRolloutRunAlways_ = false;
  simRollout.setSimulatorStateOnRolloutRunOnce_ = true;

  anymal_mrt->initRollout(&simRollout);

  switched_model::MRT_ROS_Dummy_Quadruped<JOINT_COORD_SIZE> mrt_dummy_loop(anymal_interface, *anymal_mrt,
                                                                           anymal_interface->mpcSettings().mrtDesiredFrequency_, "anymal",
                                                                           anymal_interface->mpcSettings().mpcDesiredFrequency_);

  mrt_dummy_loop.launchNodes(argc, argv);

  // initial state
  interface_t::rbd_state_vector_t initRbdState;
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
