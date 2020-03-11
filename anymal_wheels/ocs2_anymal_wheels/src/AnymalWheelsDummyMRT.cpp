/*
 * DummyMRT.cpp
 *
 *  Created on: Apr 10, 2018
 *      Author: farbod
 */

#include <ocs2_quadruped_interface/QuadrupedDummyNode.h>

#include <ocs2_mpc/MPC_Settings.h>
#include "ocs2_anymal_wheels/AnymalWheelsInterface.h"

// URDF stuff
#include <robot_state_publisher/robot_state_publisher.h>
#include <urdf/model.h>
#include <future>
#include <kdl_parser/kdl_parser.hpp>

int main(int argc, char* argv[]) {
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  const std::string taskName(argv[1]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic))

  // Initialize ros node
  ros::init(argc, argv, "anymal_wheels_mrt");
  ros::NodeHandle nodeHandle;

  // Publish wheel angles in separate thread
  // Load URDF model
  urdf::Model urdfModel;
  if (!urdfModel.initParam("ocs2_anymal_description")) {
    throw std::runtime_error("[AnymalWheelsDummyMrt] Could not read URDF from: \"ocs2_anymal_description\"");
  }
  KDL::Tree kdlTree;
  kdl_parser::treeFromUrdfModel(urdfModel, kdlTree);
  robot_state_publisher::RobotStatePublisher robotStatePublisher(kdlTree);
  auto wheelPublisher = std::async(std::launch::async, [&]() {
    ros::WallRate rate(50);
    while (ros::ok() && ros::master::check()) {
      robotStatePublisher.publishTransforms({{"LF_WHEEL", 0.0}, {"RF_WHEEL", 0.0}, {"LH_WHEEL", 0.0}, {"RH_WHEEL", 0.0}}, ros::Time::now(),
                                            "");
      rate.sleep();
    }
  });

  auto anymalInterface = anymal::getAnymalWheelsInterface(taskName);
  ocs2::MPC_Settings mpcSettings;
  mpcSettings.loadSettings(anymal::getTaskFilePathWheels(taskName));
  switched_model::quadrupedDummyNode<anymal::WheeledQuadrupedInterface>(nodeHandle, *anymalInterface, &anymalInterface->getRollout(),
                                                                        mpcSettings.mrtDesiredFrequency_, mpcSettings.mpcDesiredFrequency_);

  wheelPublisher.wait();
  return 0;
}
