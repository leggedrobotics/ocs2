#include <ocs2_anymal_raisim/AnymalRaisimConversions.h>
#include <ocs2_anymal_raisim/AnymalRaisimDummy.h>

#include <ocs2_quadruped_interface/QuadrupedDummyNode.h>
#include <ocs2_raisim/RaisimRollout.h>
#include <ocs2_raisim_ros/RaisimHeightmapRosConverter.h>
#include <ocs2_switched_model_interface/core/SwitchedModelStateEstimator.h>

namespace anymal {

void runAnymalRaisimDummy(ros::NodeHandle& nodeHandle, std::unique_ptr<switched_model::QuadrupedInterface> anymalInterface,
                          const switched_model::WholebodyDynamics<ocs2::scalar_t>& wholebodyDynamics, const std::string& urdf,
                          double mrtDesiredFreq, double mpcDesiredFreq, const ocs2::RaisimRolloutSettings& raisimRolloutSettings) {
  // Set up Raisim rollout
  anymal::AnymalRaisimConversions conversions(anymalInterface->getComModel(), anymalInterface->getKinematicModel(), wholebodyDynamics);

  ocs2::RaisimRollout simRollout(
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
  std::unique_ptr<ocs2::RaisimHeightmapRosConverter> heightmapPub;  // unique_ptr must remain alive to keep informing subscribers
  if (raisimRolloutSettings.generateTerrain_) {
    raisim::TerrainProperties terrainProperties;
    terrainProperties.zScale = raisimRolloutSettings.terrainRoughness_;
    terrainProperties.seed = raisimRolloutSettings.terrainSeed_;
    terrain = simRollout.generateTerrain(terrainProperties);
    conversions.terrain_ = terrain;
    heightmapPub.reset(new ocs2::RaisimHeightmapRosConverter());
    heightmapPub->publishGridmap(*terrain);

    auto& initState = anymalInterface->getInitialState();
    const auto initRbdState = switched_model::estimateRbdModelState(initState, switched_model::joint_coordinate_t::Zero());
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
  quadrupedDummyNode(nodeHandle, *anymalInterface, &simRollout, mrtDesiredFreq, mpcDesiredFreq);
}

}  // namespace anymal
