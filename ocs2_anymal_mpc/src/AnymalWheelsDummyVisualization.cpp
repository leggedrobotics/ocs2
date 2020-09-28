#include <ros/init.h>

// URDF stuff
#include <robot_state_publisher/robot_state_publisher.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

int main(int argc, char* argv[]) {
  // This node publishes wheel angles to be zero

  // Initialize ros node
  ros::init(argc, argv, "anymal_wheels_mrt");
  ros::NodeHandle nodeHandle;

  // Load URDF model
  urdf::Model urdfModel;
  if (!urdfModel.initParam("ocs2_anymal_description")) {
    throw std::runtime_error("[AnymalWheelsDummyMrt] Could not read URDF from: \"ocs2_anymal_description\"");
  }
  KDL::Tree kdlTree;
  kdl_parser::treeFromUrdfModel(urdfModel, kdlTree);
  robot_state_publisher::RobotStatePublisher robotStatePublisher(kdlTree);

  ros::WallRate rate(50);
  while (ros::ok() && ros::master::check()) {
    robotStatePublisher.publishTransforms({{"LF_WHEEL", 0.0}, {"RF_WHEEL", 0.0}, {"LH_WHEEL", 0.0}, {"RH_WHEEL", 0.0}}, ros::Time::now(),
                                          "");
    rate.sleep();
  }

  return 0;
}
