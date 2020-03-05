#include <ros/package.h>
#include <ros/param.h>

#include <ocs2_anymal_bear/AnymalBearInterface.h>
#include <ocs2_anymal_bear_raisim/AnymalRaisimConversions.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_quadruped_interface/QuadrupedDummyNode.h>
#include <ocs2_raisim/RaisimRollout.h>
#include <ocs2_raisim_ros/RaisimHeightmapRosConverter.h>

int main(int argc, char* argv[]) {
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  const std::string taskName(argv[1]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic))

  // Initialize ros node
  ros::init(argc, argv, "anymal_bear_mrt");
  ros::NodeHandle nodeHandle;

  // Read URDF
  const std::string urdf_param = "/anymal_bear_raisim_urdf";
  std::string urdf;
  if (!ros::param::get(urdf_param, urdf)) {
    throw ros::Exception("Error reading ros parameter: " + urdf_param);
  }

  auto anymalInterface = anymal::getAnymalBearInterface(taskName);

  // Set up Raisim rollout
  anymal::AnymalRaisimConversions conversions(anymalInterface->getComModel(), anymalInterface->getKinematicModel());
  ocs2::RaisimRolloutSettings raisimRolloutSettings;
  raisimRolloutSettings.loadSettings(ros::package::getPath("ocs2_anymal_bear_raisim") + "/config/raisim_rollout.info", "rollout");

  ocs2::RaisimRollout<switched_model::STATE_DIM, switched_model::INPUT_DIM> simRollout(
      urdf,
      std::bind(&anymal::AnymalRaisimConversions::stateToRaisimGenCoordGenVel, &conversions, std::placeholders::_1, std::placeholders::_2),
      std::bind(&anymal::AnymalRaisimConversions::raisimGenCoordGenVelToState, &conversions, std::placeholders::_1, std::placeholders::_2),
      std::bind(&anymal::AnymalRaisimConversions::inputToRaisimGeneralizedForce, &conversions, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
      std::bind(&anymal::AnymalRaisimConversions::extractModelData, &conversions, std::placeholders::_1, std::placeholders::_2),
      raisimRolloutSettings,
      std::bind(&anymal::AnymalRaisimConversions::inputToRaisimPdTargets, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));

  // Terrain
  raisim::HeightMap* terrain = nullptr;
  std::unique_ptr<ocs2::RaisimHeightmapRosConverter> heightmapPub;  // unique_ptr must remain alive until end of program
  if (raisimRolloutSettings.generateTerrain_) {
    raisim::TerrainProperties terrainProperties;
    terrainProperties.zScale = raisimRolloutSettings.terrainRoughness_;
    terrainProperties.seed = 1576493664;  // time(nullptr);
    // seed 1576493664 works well with scale 0.5
    std::cout << "terrainProperties.seed is " << terrainProperties.seed << std::endl;
    terrain = simRollout.generateTerrain(terrainProperties);  // add some random terrain
    conversions.terrain_ = terrain;
    heightmapPub.reset(new ocs2::RaisimHeightmapRosConverter());
    heightmapPub->publishGridmap(*terrain);

    auto& initState = anymalInterface->getInitialState();
    switched_model::SwitchedModelStateEstimator stateEstimator(anymalInterface->getComModel());
    const auto initRbdState = stateEstimator.estimateRbdModelState(initState, switched_model::joint_coordinate_t::Zero());
    const auto feetPositions = anymalInterface->getKinematicModel().feetPositionsInOriginFrame(
        switched_model::getBasePose(initRbdState), switched_model::getJointPositions(initRbdState));
    const Eigen::Vector3d& footPositionWithMinimumClearance =
        *std::min_element(feetPositions.begin(), feetPositions.end(), [terrain](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
          return a(2) - terrain->getHeight(a(0), a(1)) < b(2) - terrain->getHeight(b(0), b(1));
        });
    initState(5) +=
        terrain->getHeight(footPositionWithMinimumClearance(0), footPositionWithMinimumClearance(1)) - footPositionWithMinimumClearance(2);
    initState(5) -= terrain->getHeight(initState(3), initState(4));  // offset terrain height
  }

  // Run dummy loop
  ocs2::MPC_Settings mpcSettings;
  mpcSettings.loadSettings(anymal::getTaskFilePathBear(taskName));
  quadrupedDummyNode(nodeHandle, *anymalInterface, &simRollout, mpcSettings.mrtDesiredFrequency_, mpcSettings.mpcDesiredFrequency_);

  return 0;
}
