// Command
#include <ocs2_ros_interfaces/TargetPoseCommand.h>
#include <ocs2_ros_interfaces/command/TargetPoseTransformation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesJoystickInterface.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardInterface.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosInterface.h>

// Common
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

// MPC
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>

// MRT
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

// dummy target for clang toolchain
int main() {
  return 0;
}
