/*
 * AnymalTargetPoseCommand.cpp
 *
 *  Created on: Apr 15, 2018
 *      Author: farbod
 */

#include <ros/package.h>
#include <boost/program_options.hpp>

#include <ocs2_core/misc/LoadData.h>

#include "ocs2_anymal_commands/TargetTrajectories_Keyboard_Quadruped.h"

int main(int argc, char* argv[]) {
  std::string filename;
  {
    namespace po = boost::program_options;
    po::options_description desc{"Options"};
    desc.add_options()("help,h", "Help")(
        "targetcommand,t",
        po::value<std::string>(&filename)->default_value(ros::package::getPath("ocs2_anymal_commands") + "/config/targetCommand.info"),
        "TargetCommands file path");

    po::variables_map vm;
    po::store(parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
      std::cout << desc << std::endl;
      return 1;  // Fail, and exit.
    }
  }

  boost::property_tree::ptree pt;
  try {
    boost::property_tree::read_info(filename, pt);
  } catch (boost::property_tree::ptree_bad_path) {
    std::cout << "Could not find/read " << filename << "." << std::endl;
    return 2;  // Fail, and exit.
  }

  const auto targetDisplacementVelocity = pt.get<double>("targetDisplacementVelocity");
  const auto targetRotationVelocity = pt.get<double>("targetRotationVelocity");
  const auto initZHeight = pt.get<double>("comHeight");

  Eigen::Matrix<double, 12, 1> initJoints;
  ocs2::loadData::loadEigenMatrix(filename, "defaultJointState", initJoints);

  switched_model::TargetTrajectories_Keyboard_Quadruped<double, 24, 24> targetPoseCommand(
      argc, argv, "anymal", initZHeight, initJoints, targetDisplacementVelocity, targetRotationVelocity);

  targetPoseCommand.launchNodes();

  const std::string commadMsg = "Enter XYZ displacement and RollPitchYaw for the robot, separated by spaces";
  targetPoseCommand.getKeyboardCommand(commadMsg);

  // Successful exit
  return 0;
}
