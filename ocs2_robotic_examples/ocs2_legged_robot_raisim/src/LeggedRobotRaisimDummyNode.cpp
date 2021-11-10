#include <ros/init.h>
#include <ros/package.h>
#include <urdf_parser/urdf_parser.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_raisim/RaisimRollout.h>
#include <ocs2_raisim_ros/RaisimHeightmapRosConverter.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <ocs2_legged_robot/LeggedRobotInterface.h>

#include "ocs2_legged_robot_raisim/LeggedRobotRaisimConversions.h"
#include "ocs2_legged_robot_raisim/LeggedRobotRaisimVisualizer.h"

using namespace ocs2;
using namespace legged_robot;

int main(int argc, char** argv) {
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() < 5) {
    throw std::runtime_error("No robot name, config folder, target command file, or description name specified. Aborting.");
  }
  const std::string robotName(programArgs[1]);
  const std::string configName(programArgs[2]);
  const std::string targetCommandFile(programArgs[3]);
  const std::string descriptionName("/" + programArgs[4]);

  // initialize ros node
  ros::init(argc, argv, robotName + "_raisim_dummy");
  ros::NodeHandle nodeHandle;

  // legged robot interface
  std::string urdfString;
  if (!ros::param::get(descriptionName, urdfString)) {
    std::cerr << "Param " << descriptionName << " not found; unable to generate urdf" << std::endl;
  }
  LeggedRobotInterface interface(configName, targetCommandFile, urdf::parseURDF(urdfString));

  // raisim rollout
  LeggedRobotRaisimConversions conversions(interface.getPinocchioInterface(), interface.getCentroidalModelInfo(),
                                           interface.modelSettings());
  RaisimRolloutSettings raisimRolloutSettings(ros::package::getPath("ocs2_legged_robot_raisim") + "/config/raisim.info", "rollout", true);
  conversions.setGains(raisimRolloutSettings.pGains_, raisimRolloutSettings.dGains_);
  RaisimRollout raisimRollout(
      ros::package::getPath("ocs2_robotic_assets") + "/resources/anymal_c/urdf/anymal.urdf",
      ros::package::getPath("ocs2_robotic_assets") + "/resources/anymal_c/meshes",
      std::bind(&LeggedRobotRaisimConversions::stateToRaisimGenCoordGenVel, &conversions, std::placeholders::_1, std::placeholders::_2),
      std::bind(&LeggedRobotRaisimConversions::raisimGenCoordGenVelToState, &conversions, std::placeholders::_1, std::placeholders::_2),
      std::bind(&LeggedRobotRaisimConversions::inputToRaisimGeneralizedForce, &conversions, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
      nullptr, raisimRolloutSettings, nullptr);

  // terrain
  raisim::HeightMap* terrain = nullptr;
  std::unique_ptr<RaisimHeightmapRosConverter> heightmapPub;
  if (raisimRolloutSettings.generateTerrain_) {
    raisim::TerrainProperties terrainProperties;
    terrainProperties.zScale = raisimRolloutSettings.terrainRoughness_;
    terrainProperties.seed = raisimRolloutSettings.terrainSeed_;
    terrain = raisimRollout.generateTerrain(terrainProperties);
    conversions.terrain_ = terrain;
    heightmapPub.reset(new ocs2::RaisimHeightmapRosConverter());
    heightmapPub->publishGridmap(*terrain, "odom");
  }

  // mrt
  MRT_ROS_Interface mrt(robotName);
  mrt.initRollout(&raisimRollout);
  mrt.launchNodes(nodeHandle);

  // visualization
  CentroidalModelPinocchioMapping pinocchioMapping(interface.getCentroidalModelInfo());
  PinocchioEndEffectorKinematics endEffectorKinematics(interface.getPinocchioInterface(), pinocchioMapping,
                                                       interface.modelSettings().contactNames3DoF);
  std::shared_ptr<LeggedRobotRaisimVisualizer> leggedRobotRaisimVisualizer(new LeggedRobotRaisimVisualizer(
      interface.getPinocchioInterface(), interface.getCentroidalModelInfo(), endEffectorKinematics, nodeHandle, 100.0, terrain));

  // legged robot dummy
  MRT_ROS_Dummy_Loop leggedRobotDummy(mrt, interface.mpcSettings().mrtDesiredFrequency_, interface.mpcSettings().mpcDesiredFrequency_);
  leggedRobotDummy.subscribeObservers({leggedRobotRaisimVisualizer});

  // initial state
  SystemObservation initObservation;
  initObservation.mode = ModeNumber::STANCE;
  initObservation.time = 0.0;
  initObservation.state = interface.getInitialState();
  initObservation.input = vector_t::Zero(interface.getCentroidalModelInfo().inputDim);

  // initial command
  TargetTrajectories initTargetTrajectories({initObservation.time}, {initObservation.state}, {initObservation.input});

  // run dummy
  leggedRobotDummy.run(initObservation, initTargetTrajectories);

  return 0;
}