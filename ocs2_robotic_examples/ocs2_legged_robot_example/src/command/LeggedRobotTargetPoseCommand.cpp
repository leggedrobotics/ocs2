#include <ocs2_core/misc/LoadData.h>
#include <ros/package.h>

#include <ocs2_legged_robot_example/command/TargetTrajectoriesKeyboardQuadruped.h>
#include <ocs2_legged_robot_example/common/definitions.h>

using namespace ocs2;
using namespace legged_robot;

int main(int argc, char* argv[]) {
  std::string filename = ROBOT_COMMAND_PATH_;
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  using quadrupedKeyboard = TargetTrajectoriesKeyboardQuadruped;
  const auto targetDisplacementVelocity = pt.get<double>("targetDisplacementVelocity");
  const auto targetRotationVelocity = pt.get<double>("targetRotationVelocity");
  const auto initZHeight = pt.get<double>("comHeight");
  vector_t initJoints(centroidalModelInfo.actuatedDofNum);
  ocs2::loadData::loadEigenMatrix(filename, "defaultJointState", initJoints);

  quadrupedKeyboard targetPoseCommand(argc, argv, ROBOT_NAME_, initZHeight, initJoints, targetDisplacementVelocity, targetRotationVelocity);

  targetPoseCommand.launchNodes();

  const std::string commandMsg = "Enter XYZ linear and angular displacements (in degrees) for the TORSO, separated by spaces";
  targetPoseCommand.getKeyboardCommand(commandMsg);

  // Successful exit
  return 0;
}
