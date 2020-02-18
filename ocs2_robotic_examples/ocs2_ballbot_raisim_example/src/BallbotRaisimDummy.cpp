#include <ros/package.h>

#include <ocs2_ballbot_example/BallbotInterface.h>
#include <ocs2_ballbot_example/definitions.h>
#include <ocs2_ballbot_example/ros_comm/BallbotDummyVisualization.h>
#include <ocs2_ballbot_raisim_example/BallbotRaisimConversions.h>
#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_raisim/RaisimRollout.h>
#include <ocs2_raisim_ros/RaisimHeightmapRosConverter.h>

int main(int argc, char* argv[]) {
  const std::string robotName = "ballbot";
  using mrt_t = ocs2::MRT_ROS_Interface<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_>;
  using dummy_t = ocs2::MRT_ROS_Dummy_Loop<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_>;
  using vis_t = ocs2::ballbot::BallbotDummyVisualization;

  // task file
  if (argc <= 1) {
    throw std::runtime_error("No task tile specified.");
  }
  const std::string taskFileFolderName(argv[1]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

  // Init ros
  ros::init(argc, argv, robotName + "_raisim_dummy");
  ros::NodeHandle nodeHandle;

  // read urdf file
  const std::string urdfParamName = "/ocs2_ballbot_raisim_description";
  std::string urdf;
  if (!ros::param::get(urdfParamName, urdf)) {
    throw ros::Exception("Error reading urdf from parameter server: " + urdfParamName);
  }

  // setup raisim rollout
  ocs2::RaisimRolloutSettings raisimRolloutSettings(ros::package::getPath("ocs2_ballbot_raisim_example") + "/config/raisim_rollout.info",
                                                    "rollout", true);
  ocs2::ballbot::BallbotRaisimConversions conversions;
  ocs2::RaisimRollout<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_> simRollout(
      urdf,
      std::bind(&ocs2::ballbot::BallbotRaisimConversions::stateToRaisimGenCoordGenVel, &conversions, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&ocs2::ballbot::BallbotRaisimConversions::raisimGenCoordGenVelToState, &conversions, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&ocs2::ballbot::BallbotRaisimConversions::inputToRaisimGeneralizedForce, &conversions, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
      nullptr, raisimRolloutSettings, nullptr);

  // Terrain
  std::unique_ptr<ocs2::RaisimHeightmapRosConverter> heightmapPub;  // keep object alive until end of program
  if (raisimRolloutSettings.generateTerrain_) {
    raisim::TerrainProperties terrainProperties;
    terrainProperties.zScale = raisimRolloutSettings.terrainRoughness_;
    terrainProperties.seed = 1;
    auto terrain = simRollout.generateTerrain(terrainProperties);

    // ensure height at zero is zero
    const auto heightAtZero = terrain->getHeight(0.0, 0.0);
    for (auto& height : terrain->getHeightMap()) {
      height -= heightAtZero;
    }

    heightmapPub.reset(new ocs2::RaisimHeightmapRosConverter());
    heightmapPub->publishGridmap(*terrain);
  }

  // setup MRT
  mrt_t mrt(robotName);
  mrt.initRollout(&simRollout);
  mrt.launchNodes(nodeHandle);

  // Visualization
  std::shared_ptr<vis_t> ballbotDummyVisualization(new vis_t(nodeHandle));

  // Dummy
  ocs2::ballbot::BallbotInterface interface(taskFileFolderName);
  dummy_t dummyBallbot(mrt, interface.mpcSettings().mrtDesiredFrequency_, interface.mpcSettings().mpcDesiredFrequency_);
  dummyBallbot.subscribeObservers({ballbotDummyVisualization});

  // initial state and command
  mrt_t::system_observation_t initObservation;
  initObservation.state() = interface.getInitialState();

  // initial command
  ocs2::CostDesiredTrajectories initCostDesiredTrajectories({initObservation.time()}, {initObservation.state()}, {initObservation.input()});

  // run dummy
  dummyBallbot.run(initObservation, initCostDesiredTrajectories);

  return 0;
}
