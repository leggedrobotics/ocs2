
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "ocs2_anymal_commands/MotionCommandController.h"
#include "ocs2_anymal_commands/MotionCommandDummy.h"

int main(int argc, char *argv[])
{
  const std::string robotName = "anymal";
  std::string motionFile = ament_index_cpp::get_package_share_directory("ocs2_anymal_commands") + "/config/motions.info";
  std::cerr << "Loading motion file: " << motionFile << std::endl;

  const std::string controllerName = [&]
  {
    std::vector<std::string> programArgs = rclcpp::remove_ros_arguments(argc, argv);
    
    if (programArgs.size() <= 1)
    {
      throw std::runtime_error("No operation mode specified. Aborting.");
    }
    return programArgs[1];
  }();

  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(robotName + "_mpc_motion_command");

  std::unique_ptr<switched_model::MotionCommandInterface> motionCommandInterface;
  if (controllerName == "dummy")
  {
    motionCommandInterface.reset(new switched_model::MotionCommandDummy(node, motionFile, robotName));
  }
  else
  {
    motionCommandInterface.reset(new switched_model::MotionCommandController(node, motionFile, controllerName));
  }

  rclcpp::Rate rate(10);
  while (rclcpp::ok())
  {
    motionCommandInterface->getKeyboardCommand();
    rate.sleep();
  }

  // Successful exit
  return 0;
}
