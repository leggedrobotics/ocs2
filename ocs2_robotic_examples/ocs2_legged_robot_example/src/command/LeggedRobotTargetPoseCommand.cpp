#include <ocs2_core/misc/LoadData.h>

#include <ocs2_legged_robot_example/command/TargetTrajectoriesKeyboardQuadruped.h>

using namespace ocs2;
using namespace legged_robot;

int main(int argc, char* argv[]) {
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() < 3) {
    throw std::runtime_error("No robot name or target command file specified. Aborting.");
  }
  const std::string robotName(programArgs[1]);
  const std::string targetCommandFile(programArgs[2]);

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(targetCommandFile, pt);

  using quadrupedKeyboard = TargetTrajectoriesKeyboardQuadruped;
  const auto targetDisplacementVelocity = pt.get<double>("targetDisplacementVelocity");
  const auto targetRotationVelocity = pt.get<double>("targetRotationVelocity");
  const auto initZHeight = pt.get<double>("comHeight");
  vector_t initJoints(quadrupedKeyboard::actuatedDofNum_);
  ocs2::loadData::loadEigenMatrix(targetCommandFile, "defaultJointState", initJoints);

  quadrupedKeyboard targetPoseCommand(argc, argv, robotName, initZHeight, initJoints, targetDisplacementVelocity, targetRotationVelocity);

  targetPoseCommand.launchNodes();

  const std::string commandMsg = "Enter XYZ linear and angular displacements (in degrees) for the TORSO, separated by spaces";
  targetPoseCommand.getKeyboardCommand(commandMsg);

  // Successful exit
  return 0;
}
