/******************************************************************************
Copyright (c) 2022, Farbod Farshidian. All rights reserved.

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

#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_legged_robot_raisim/LeggedRobotRaisimConversions.h>
#include <ocs2_legged_robot_raisim/LeggedRobotRaisimVisualizer.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_mpcnet_core/control/MpcnetOnnxController.h>
#include <ocs2_mpcnet_core/dummy/MpcnetDummyLoopRos.h>
#include <ocs2_mpcnet_core/dummy/MpcnetDummyObserverRos.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_raisim_core/RaisimRollout.h>
#include <ocs2_raisim_ros/RaisimHeightmapRosConverter.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

#include "ocs2_legged_robot_mpcnet/LeggedRobotMpcnetDefinition.h"

using namespace ocs2;
using namespace legged_robot;

int main(int argc, char** argv) {
  const std::string robotName = "legged_robot";

  // initialize ros node
  ros::init(argc, argv, robotName + "_mpcnet_dummy");
  ros::NodeHandle nodeHandle;
  // Get node parameters
  bool useRaisim;
  std::string taskFile, urdfFile, referenceFile, raisimFile, resourcePath, policyFile;
  nodeHandle.getParam("/taskFile", taskFile);
  nodeHandle.getParam("/urdfFile", urdfFile);
  nodeHandle.getParam("/referenceFile", referenceFile);
  nodeHandle.getParam("/raisimFile", raisimFile);
  nodeHandle.getParam("/resourcePath", resourcePath);
  nodeHandle.getParam("/policyFile", policyFile);
  nodeHandle.getParam("/useRaisim", useRaisim);

  // legged robot interface
  LeggedRobotInterface leggedRobotInterface(taskFile, urdfFile, referenceFile);

  // gait receiver
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(nodeHandle, leggedRobotInterface.getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);

  // ROS reference manager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, leggedRobotInterface.getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nodeHandle);

  // policy (MPC-Net controller)
  auto onnxEnvironmentPtr = ocs2::mpcnet::createOnnxEnvironment();
  auto mpcnetDefinitionPtr = std::make_shared<LeggedRobotMpcnetDefinition>(leggedRobotInterface);
  auto mpcnetControllerPtr =
      std::make_unique<ocs2::mpcnet::MpcnetOnnxController>(mpcnetDefinitionPtr, rosReferenceManagerPtr, onnxEnvironmentPtr);
  mpcnetControllerPtr->loadPolicyModel(policyFile);

  // rollout
  std::unique_ptr<RolloutBase> rolloutPtr;
  raisim::HeightMap* terrainPtr = nullptr;
  std::unique_ptr<RaisimHeightmapRosConverter> heightmapPub;
  std::unique_ptr<LeggedRobotRaisimConversions> conversions;
  if (useRaisim) {
    conversions.reset(new LeggedRobotRaisimConversions(leggedRobotInterface.getPinocchioInterface(),
                                                       leggedRobotInterface.getCentroidalModelInfo(),
                                                       leggedRobotInterface.getInitialState()));
    RaisimRolloutSettings raisimRolloutSettings(raisimFile, "rollout", true);
    conversions->loadSettings(raisimFile, "rollout", true);
    rolloutPtr.reset(new RaisimRollout(
        urdfFile, resourcePath,
        [&](const vector_t& state, const vector_t& input) { return conversions->stateToRaisimGenCoordGenVel(state, input); },
        [&](const Eigen::VectorXd& q, const Eigen::VectorXd& dq) { return conversions->raisimGenCoordGenVelToState(q, dq); },
        [&](double time, const vector_t& input, const vector_t& state, const Eigen::VectorXd& q, const Eigen::VectorXd& dq) {
          return conversions->inputToRaisimGeneralizedForce(time, input, state, q, dq);
        },
        nullptr, raisimRolloutSettings,
        [&](double time, const vector_t& input, const vector_t& state, const Eigen::VectorXd& q, const Eigen::VectorXd& dq) {
          return conversions->inputToRaisimPdTargets(time, input, state, q, dq);
        }));
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
  auto mpcnetDummyObserverRosPtr = std::make_shared<ocs2::mpcnet::MpcnetDummyObserverRos>(nodeHandle, robotName);

  // visualization
  CentroidalModelPinocchioMapping pinocchioMapping(leggedRobotInterface.getCentroidalModelInfo());
  PinocchioEndEffectorKinematics endEffectorKinematics(leggedRobotInterface.getPinocchioInterface(), pinocchioMapping,
                                                       leggedRobotInterface.modelSettings().contactNames3DoF);
  std::shared_ptr<LeggedRobotVisualizer> leggedRobotVisualizerPtr;
  if (useRaisim) {
    leggedRobotVisualizerPtr.reset(new LeggedRobotRaisimVisualizer(
        leggedRobotInterface.getPinocchioInterface(), leggedRobotInterface.getCentroidalModelInfo(), endEffectorKinematics, nodeHandle));
    static_cast<LeggedRobotRaisimVisualizer*>(leggedRobotVisualizerPtr.get())->updateTerrain();
  } else {
    leggedRobotVisualizerPtr.reset(new LeggedRobotVisualizer(
        leggedRobotInterface.getPinocchioInterface(), leggedRobotInterface.getCentroidalModelInfo(), endEffectorKinematics, nodeHandle));
  }

  // MPC-Net dummy loop ROS
  const scalar_t controlFrequency = leggedRobotInterface.mpcSettings().mrtDesiredFrequency_;
  const scalar_t rosFrequency = leggedRobotInterface.mpcSettings().mpcDesiredFrequency_;
  ocs2::mpcnet::MpcnetDummyLoopRos mpcnetDummyLoopRos(controlFrequency, rosFrequency, std::move(mpcnetControllerPtr), std::move(rolloutPtr),
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
  const TargetTrajectories targetTrajectories({systemObservation.time}, {systemObservation.state}, {systemObservation.input});

  // run MPC-Net dummy loop ROS
  mpcnetDummyLoopRos.run(systemObservation, targetTrajectories);

  // successful exit
  return 0;
}
