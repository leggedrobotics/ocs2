#include <ros/package.h>

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <ocs2_raisim/RaisimRollout.h>

#include <ocs2_quadruped_interface/QuadrupedXppVisualizer.h>

#include <ocs2_anymal_bear/AnymalBearInterface.h>

#include <ocs2_anymal_bear_raisim/AnymalRaisimConversions.h>

int main(int argc, char* argv[]) {
  constexpr int STATE_DIM = 24;
  constexpr int INPUT_DIM = 24;
  constexpr int JOINT_COORD_SIZE = 12;

  const std::string robotName = "anymal";
  using interface_t = anymal::AnymalBearInterface;
  using vis_t = switched_model::QuadrupedXppVisualizer<JOINT_COORD_SIZE>;
  using mrt_t = ocs2::MRT_ROS_Interface<STATE_DIM, INPUT_DIM>;
  using dummy_t = ocs2::MRT_ROS_Dummy_Loop<STATE_DIM, INPUT_DIM>;

  // task file
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFileFolderName = std::string(argv[1]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

  ros::init(argc, argv, "anymal_visualizer");
  ros::NodeHandle n;

  interface_t anymalBearInterface(ros::package::getPath("ocs2_anymal_bear") + "/config/" + taskFileFolderName);

  const std::string urdf_param = "/ocs2_anymal_bear_description";
  std::string urdf;
  if (!ros::param::get(urdf_param, urdf)) {
    throw ros::Exception("Error reading ros parameter: " + urdf_param);
  }

  // MRT with simulator rollout
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

  mrt_t mrt(robotName);
  mrt.initRollout(&simRollout);
  mrt.launchNodes(n);

  // Visualization
  std::shared_ptr<vis_t> visualizer(new vis_t(anymalBearInterface.getKinematicModel(), anymalBearInterface.getComModel(), robotName, n));

  // Dummy MRT
  dummy_t dummySimulator(mrt, anymalBearInterface.mpcSettings().mrtDesiredFrequency_,
                         anymalBearInterface.mpcSettings().mpcDesiredFrequency_);
  dummySimulator.subscribeObservers({visualizer});

  // initial state
  mrt_t::system_observation_t initObservation;
  initObservation.state() = anymalBearInterface.getInitialState();
  initObservation.subsystem() = switched_model::ModeNumber::STANCE;

  // initial command
  ocs2::CostDesiredTrajectories initCostDesiredTrajectories({0.0}, {initObservation.state()}, {initObservation.input()});

  // run dummy
  dummySimulator.run(initObservation, initCostDesiredTrajectories);

  return 0;
}
