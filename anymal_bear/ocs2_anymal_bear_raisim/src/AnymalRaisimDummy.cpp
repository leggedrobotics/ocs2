#include <ros/package.h>

#include <ocs2_anymal_bear/AnymalBearInterface.h>
#include <ocs2_anymal_bear_raisim/AnymalRaisimConversions.h>
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

  const std::string urdf_param = "/ocs2_anymal_bear_description";
  std::string urdf;
  if (!ros::param::get(urdf_param, urdf)) {
    throw ros::Exception("Error reading ros parameter: " + urdf_param);
  }

  // setup MRT with simulator rollouts
  using mrt_t = switched_model::MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>;
  std::shared_ptr<mrt_t> anymal_mrt(new mrt_t(anymal_interface, "anymal"));

  anymal::AnymalRaisimConversions conversions;

  ocs2::RaisimRolloutSettings raisimRolloutSettings;
  raisimRolloutSettings.loadSettings(ros::package::getPath("ocs2_anymal_bear_raisim") + "/config/raisim_rollout.info", "rollout");

  ocs2::RaisimRollout<STATE_DIM, INPUT_DIM> simRollout(
      urdf,
      std::bind(&anymal::AnymalRaisimConversions::stateToRaisimGenCoordGenVel, &conversions, std::placeholders::_1, std::placeholders::_2),
      std::bind(&anymal::AnymalRaisimConversions::raisimGenCoordGenVelToState, &conversions, std::placeholders::_1, std::placeholders::_2),
      std::bind(&anymal::AnymalRaisimConversions::inputToRaisimGeneralizedForce, &conversions, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
      std::bind(&anymal::AnymalRaisimConversions::extractModelData, &conversions, std::placeholders::_1, std::placeholders::_2),
      raisimRolloutSettings,
      std::bind(&anymal::AnymalRaisimConversions::inputToRaisimPdTargets, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));

  anymal_mrt->initRollout(&simRollout);

  // Visualizer
  std::unique_ptr<switched_model::QuadrupedXppVisualizer<12>> visualizer(new switched_model::QuadrupedXppVisualizer<12>(
      anymal_interface->getKinematicModel(), anymal_interface->getComModel(), "anymal", true));

  // Dummy MRT
  switched_model::MRT_ROS_Dummy_Quadruped<JOINT_COORD_SIZE> mrt_dummy_loop(std::move(visualizer), *anymal_mrt,
                                                                           anymal_interface->mpcSettings().mrtDesiredFrequency_,
                                                                           anymal_interface->mpcSettings().mpcDesiredFrequency_);

  mrt_dummy_loop.launchNodes(argc, argv);

  // initial state
  mrt_t::system_observation_t initObservation;
  initObservation.time() = 0.0;
  initObservation.state() = anymal_interface->getInitialState();
  initObservation.input().setZero();
  initObservation.subsystem() = 15;

  // initial command
  ocs2::CostDesiredTrajectories initCostDesiredTrajectories({0.0}, {initObservation.state()}, {initObservation.input()});

  mrt_dummy_loop.run(initObservation, initCostDesiredTrajectories);

  return 0;
}
