// command
#include <ocs2_ros_interfaces/command/TargetTrajectoriesInteractiveMarker.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardPublisher.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

// common
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>

// mpc
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>

// mrt
#include <ocs2_ros_interfaces/mrt/LoopshapingDummyObserver.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

// synchronized_module
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

// dummy target for clang toolchain
int main() {
  return 0;
}
