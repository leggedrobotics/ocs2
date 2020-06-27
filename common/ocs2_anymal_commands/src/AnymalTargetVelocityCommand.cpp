/*
 * AnymalTargetPoseCommand.cpp
 *
 *  Created on: Apr 15, 2018
 *      Author: farbod
 */

#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_core/misc/LoadData.h>

#include "ocs2_anymal_commands/TargetTrajectories_Keyboard_Quadruped.h"

int main(int argc, char* argv[]) {
  using scalar_t = ocs2::scalar_t;
  std::string filename;

  {
    std::vector<std::string> programArgs{};
    ros::removeROSArgs(argc, argv, programArgs);
    if (programArgs.size() <= 1)
      throw std::runtime_error("No task file specified. Aborting.");
    else
      filename = programArgs[1];
  }

  boost::property_tree::ptree pt;
  try {
    boost::property_tree::read_info(filename, pt);
  } catch (boost::property_tree::ptree_bad_path) {
    std::cout << "Could not find/read " << filename << "." << std::endl;
    return 1;  // Fail, and exit.
  }

  const auto targetDisplacementVelocity = pt.get<scalar_t>("targetDisplacementVelocity");
  const auto targetRotationVelocity = pt.get<scalar_t>("targetRotationVelocity");
  const auto initZHeight = pt.get<scalar_t>("comHeight");

  Eigen::Matrix<scalar_t, 12, 1> initJoints;
  ocs2::loadData::loadEigenMatrix(filename, "defaultJointState", initJoints);

  using quadrupedKeyboard = switched_model::TargetTrajectories_Keyboard_Quadruped;
  auto command_mode = quadrupedKeyboard::COMMAND_MODE::VELOCITY;
  std::vector<scalar_t> command_limits{2.0, 2.0, 0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 40.0, 40.0, 360.0};

  quadrupedKeyboard targetPoseCommand(argc, argv, "anymal", initZHeight, initJoints, targetDisplacementVelocity, targetRotationVelocity,
                                      command_limits, command_mode);

  targetPoseCommand.launchNodes();

  const std::string commadMsg = "Enter XYZ Velocity and Angular velocities for the robot, separated by spaces";
  targetPoseCommand.getKeyboardCommand(commadMsg);

  // Successful exit
  return 0;
}
