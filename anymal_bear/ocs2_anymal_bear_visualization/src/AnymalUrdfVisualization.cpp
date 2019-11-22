

#include <ros/ros.h>

#include <xpp_states/endeffector_mappings.h>

#include <xpp_msgs/topic_names.h>
#include <xpp_vis/urdf_visualizer.h>

using namespace xpp;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "ocs2_anymal_bear_visualization_visualizer");
  ros::NodeHandle nh;

  const std::string joint_topic = "xpp/joint_anymal_des";
  std::string urdf_param = "ocs2_anymal_bear_description";

  std::string urdf;
  if (!ros::param::get(urdf_param, urdf)) {
    throw ros::Exception("Error reading ros parameter: " + urdf_param);
  }

  // urdf joint names
  int n_ee = 4;
  int n_j = 3;  // degrees of freedom per leg
  std::vector<UrdfVisualizer::URDFName> joint_names(n_ee * n_j);

  // to perform addition
  int HAA = 0;
  int HFE = 1;
  int KFE = 2;

  // left: index in xpp framework, right: joint-names in urdf file
  joint_names.at(n_j * quad::LF + HAA) = "LF_HAA";
  joint_names.at(n_j * quad::LF + HFE) = "LF_HFE";
  joint_names.at(n_j * quad::LF + KFE) = "LF_KFE";
  joint_names.at(n_j * quad::RF + HAA) = "RF_HAA";
  joint_names.at(n_j * quad::RF + HFE) = "RF_HFE";
  joint_names.at(n_j * quad::RF + KFE) = "RF_KFE";
  joint_names.at(n_j * quad::LH + HAA) = "LH_HAA";
  joint_names.at(n_j * quad::LH + HFE) = "LH_HFE";
  joint_names.at(n_j * quad::LH + KFE) = "LH_KFE";
  joint_names.at(n_j * quad::RH + HAA) = "RH_HAA";
  joint_names.at(n_j * quad::RH + HFE) = "RH_HFE";
  joint_names.at(n_j * quad::RH + KFE) = "RH_KFE";

  UrdfVisualizer anymal_desired(urdf_param, joint_names, "base", "world", joint_topic, "anymal_des");
  UrdfVisualizer anymal_current(urdf_param, joint_names, "base", "world", joint_topic, "anymal_curr");

  ros::spin();

  return 0;
}
