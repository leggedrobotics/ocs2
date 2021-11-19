#include <ros/init.h>
#include <ros/package.h>
#include <urdf_parser/urdf_parser.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_legged_robot/gait/GaitReceiver.h>
#include <ocs2_legged_robot/visualization/LeggedRobotVisualizer.h>
#include <ocs2_legged_robot_raisim/LeggedRobotRaisimConversions.h>
#include <ocs2_legged_robot_raisim/LeggedRobotRaisimVisualizer.h>
#include <ocs2_mpcnet/control/MpcnetOnnxController.h>
#include <ocs2_mpcnet/dummy/MpcnetDummyLoopRos.h>
#include <ocs2_mpcnet/dummy/MpcnetDummyObserverRos.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_raisim/RaisimRollout.h>
#include <ocs2_raisim_ros/RaisimHeightmapRosConverter.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

#include "ocs2_legged_robot_mpcnet/LeggedRobotMpcnetDefinition.h"

using namespace ocs2;
using namespace legged_robot;

int main(int argc, char** argv) {
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() < 7) {
    throw std::runtime_error(
        "No robot name, config folder, target command file, description file, policy file path, or rollout type specified. Aborting.");
  }
  const std::string robotName(programArgs[1]);
  const std::string configName(programArgs[2]);
  const std::string targetCommandFile(programArgs[3]);
  const std::string descriptionFile(programArgs[4]);
  const std::string policyFilePath(programArgs[5]);
  const bool raisim = (programArgs[6] == "true") ? true : false;

  // initialize ros node
  ros::init(argc, argv, robotName + "_mpcnet_dummy");
  ros::NodeHandle nodeHandle;

  // legged robot interface
  LeggedRobotInterface leggedRobotInterface(configName, targetCommandFile, urdf::parseURDFFile(descriptionFile));

  // gait receiver
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(nodeHandle, leggedRobotInterface.getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);

  // ROS reference manager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, leggedRobotInterface.getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nodeHandle);

  // policy (MPC-Net controller)
  auto onnxEnvironmentPtr = createOnnxEnvironment();
  std::shared_ptr<MpcnetDefinitionBase> mpcnetDefinitionPtr(new LeggedRobotMpcnetDefinition(leggedRobotInterface.getInitialState()));
  std::unique_ptr<MpcnetControllerBase> mpcnetControllerPtr(
      new MpcnetOnnxController(mpcnetDefinitionPtr, rosReferenceManagerPtr, onnxEnvironmentPtr));
  mpcnetControllerPtr->loadPolicyModel(policyFilePath);

  // rollout
  std::unique_ptr<RolloutBase> rolloutPtr;
  raisim::HeightMap* terrainPtr = nullptr;
  std::unique_ptr<RaisimHeightmapRosConverter> heightmapPub;
  std::unique_ptr<LeggedRobotRaisimConversions> conversions;
  if (raisim) {
    conversions.reset(
        new LeggedRobotRaisimConversions(leggedRobotInterface.getPinocchioInterface(), leggedRobotInterface.getCentroidalModelInfo()));
    RaisimRolloutSettings raisimRolloutSettings(ros::package::getPath("ocs2_legged_robot_raisim") + "/config/raisim.info", "rollout", true);
    conversions->setGains(raisimRolloutSettings.pGains_, raisimRolloutSettings.dGains_);
    rolloutPtr.reset(new RaisimRollout(
        ros::package::getPath("ocs2_robotic_assets") + "/resources/anymal_c/urdf/anymal.urdf",
        ros::package::getPath("ocs2_robotic_assets") + "/resources/anymal_c/meshes",
        [&](const vector_t& state, const vector_t& input) { return conversions->stateToRaisimGenCoordGenVel(state, input); },
        [&](const Eigen::VectorXd& q, const Eigen::VectorXd& dq) { return conversions->raisimGenCoordGenVelToState(q, dq); },
        [&](double time, const vector_t& input, const vector_t& state, const Eigen::VectorXd& q, const Eigen::VectorXd& dq) {
          return conversions->inputToRaisimGeneralizedForce(time, input, state, q, dq);
        },
        nullptr, raisimRolloutSettings, nullptr));
    // terrain
    if (raisimRolloutSettings.generateTerrain_) {
      raisim::TerrainProperties terrainProperties;
      terrainProperties.zScale = raisimRolloutSettings.terrainRoughness_;
      terrainProperties.seed = raisimRolloutSettings.terrainSeed_;
      terrainPtr = static_cast<RaisimRollout*>(rolloutPtr.get())->generateTerrain(terrainProperties);
      conversions->setTerrain(*terrainPtr);
      heightmapPub.reset(new ocs2::RaisimHeightmapRosConverter());
      heightmapPub->publishGridmap(*terrainPtr, "odom");
    }
  } else {
    rolloutPtr.reset(leggedRobotInterface.getRollout().clone());
  }

  // observer
  std::shared_ptr<MpcnetDummyObserverRos> mpcnetDummyObserverRosPtr(new MpcnetDummyObserverRos(nodeHandle, robotName));

  // visualization
  CentroidalModelPinocchioMapping pinocchioMapping(leggedRobotInterface.getCentroidalModelInfo());
  PinocchioEndEffectorKinematics endEffectorKinematics(leggedRobotInterface.getPinocchioInterface(), pinocchioMapping,
                                                       leggedRobotInterface.modelSettings().contactNames3DoF);
  std::shared_ptr<LeggedRobotVisualizer> leggedRobotVisualizerPtr;
  if (raisim) {
    leggedRobotVisualizerPtr.reset(new LeggedRobotRaisimVisualizer(
        leggedRobotInterface.getPinocchioInterface(), leggedRobotInterface.getCentroidalModelInfo(), endEffectorKinematics, nodeHandle));
    static_cast<LeggedRobotRaisimVisualizer*>(leggedRobotVisualizerPtr.get())->updateTerrain();
  } else {
    leggedRobotVisualizerPtr.reset(new LeggedRobotVisualizer(
        leggedRobotInterface.getPinocchioInterface(), leggedRobotInterface.getCentroidalModelInfo(), endEffectorKinematics, nodeHandle));
  }

  // MPC-Net dummy loop ROS
  scalar_t controlFrequency = leggedRobotInterface.mpcSettings().mrtDesiredFrequency_;
  scalar_t rosFrequency = leggedRobotInterface.mpcSettings().mpcDesiredFrequency_;
  MpcnetDummyLoopRos mpcnetDummyLoopRos(controlFrequency, rosFrequency, std::move(mpcnetControllerPtr), std::move(rolloutPtr),
                                        rosReferenceManagerPtr);
  mpcnetDummyLoopRos.addObserver(mpcnetDummyObserverRosPtr);
  mpcnetDummyLoopRos.addObserver(leggedRobotVisualizerPtr);
  mpcnetDummyLoopRos.addSynchronizedModule(gaitReceiverPtr);

  // initial system observation
  SystemObservation systemObservation;
  systemObservation.mode = ModeNumber::STANCE;
  systemObservation.time = 0.0;
  systemObservation.state = leggedRobotInterface.getInitialState();
  systemObservation.input = vector_t::Zero(leggedRobotInterface.getCentroidalModelInfo().inputDim);

  // initial target trajectories
  TargetTrajectories targetTrajectories({systemObservation.time}, {systemObservation.state}, {systemObservation.input});

  // run MPC-Net dummy loop ROS
  mpcnetDummyLoopRos.run(systemObservation, targetTrajectories);

  // successful exit
  return 0;
}
