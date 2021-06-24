
#include <ocs2_legged_robot_example/common/utils.h>
#include <ocs2_legged_robot_example/visualization/LeggedRobotDummyVisualization.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <array>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/MarkerArray.h>

namespace ocs2 {
namespace legged_robot {

void LeggedRobotDummyVisualization::launchVisualizerNode(ros::NodeHandle& nodeHandle) {
  tfBroadcasterPtr_.reset(new tf::TransformBroadcaster);

  // load a kdl-tree from the urdf robot description and initialize the robot state publisher
  std::string urdfName = "legged_robot_description";
  urdf::Model model;
  if (!model.initParam(urdfName)) ROS_ERROR("URDF model load was NOT successful");
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
  }
  robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(tree));
  robotStatePublisherPtr_->publishFixedTransforms(true);

  policyDelayPublisher_ = nodeHandle.advertise<std_msgs::Float64>(ocs2::legged_robot::ROBOT_NAME_ + "_mpc_policy_delay", 1);
  stateOptimizedPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/legged_robot/optimizedStateTrajectory", 1);

  for (int k = 2; k < pinocchioInterface_.getModel().njoints; ++k) {
    jointPositions_.insert(std::pair<std::string, scalar_t>(pinocchioInterface_.getModel().names[k], 0.0));
  }
}

void LeggedRobotDummyVisualization::update(const SystemObservation& observation, const PrimalSolution& policy, const CommandData& command) {
  const ros::Time timeStamp = ros::Time::now();

  elapsedTime_ += dt_;
  policyDelayFromObservation_ = elapsedTime_ - command.mpcInitObservation_.time;
  std_msgs::Float64 rosMsg;
  rosMsg.data = policyDelayFromObservation_;
  policyDelayPublisher_.publish(rosMsg);

  Eigen::Matrix<scalar_t, ACTUATED_DOF_NUM_, 1> q = observation.state.segment(2 * BASE_DOF_NUM_, ACTUATED_DOF_NUM_);
  for (int k = 2; k < pinocchioInterface_.getModel().njoints; ++k) {
    jointPositions_[pinocchioInterface_.getModel().names[k]] = q(k - 2);
  }
  robotStatePublisherPtr_->publishTransforms(jointPositions_, timeStamp);

  publishBaseTransform(timeStamp, observation);
  publishOptimizedStateTrajectory(timeStamp, policy.stateTrajectory_);
}

/* helpers taken from ocs2_quadruped_interface/QuadrupedVisualizationHelpers */

static std_msgs::ColorRGBA getColor(std::array<scalar_t, 3> rgb, scalar_t alpha = 1.0) {
  std_msgs::ColorRGBA colorMsg;
  colorMsg.r = rgb[0];
  colorMsg.g = rgb[1];
  colorMsg.b = rgb[2];
  colorMsg.a = alpha;
  return colorMsg;
}

static geometry_msgs::Quaternion getOrientationMsg(const Eigen::Quaterniond& orientation) {
  geometry_msgs::Quaternion orientationMsg;
  orientationMsg.x = orientation.x();
  orientationMsg.y = orientation.y();
  orientationMsg.z = orientation.z();
  orientationMsg.w = orientation.w();
  return orientationMsg;
}

static visualization_msgs::Marker getLineMsg(std::vector<geometry_msgs::Point>&& points, std::array<scalar_t, 3> color,
                                             scalar_t lineWidth) {
  visualization_msgs::Marker line;
  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.scale.x = lineWidth;
  line.color = getColor(color);
  line.points = std::move(points);
  line.pose.orientation = getOrientationMsg({1., 0., 0., 0.});
  return line;
}

static Eigen::Quaternion<scalar_t> quaternionBaseToOrigin(const Eigen::Matrix<scalar_t, 3, 1>& eulerAngles) {
  const auto roll = eulerAngles(0);
  const auto pitch = eulerAngles(1);
  const auto yaw = eulerAngles(2);
  return Eigen::AngleAxis<scalar_t>(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxis<scalar_t>(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxis<scalar_t>(yaw, Eigen::Vector3d::UnitZ());
}

static geometry_msgs::Point getPointMsg(const Eigen::Vector3d& point) {
  geometry_msgs::Point pointMsg;
  pointMsg.x = point.x();
  pointMsg.y = point.y();
  pointMsg.z = point.z();
  return pointMsg;
}

static std_msgs::Header getHeaderMsg(const std::string& frame_id, const ros::Time& timeStamp) {
  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = timeStamp;
  return header;
}

template <typename It>
void assignHeader(It firstIt, It lastIt, const std_msgs::Header& header) {
  for (; firstIt != lastIt; ++firstIt) {
    firstIt->header = header;
  }
}

template <typename It>
void assignIncreasingId(It firstIt, It lastIt, int startId = 0) {
  for (; firstIt != lastIt; ++firstIt) {
    firstIt->id = startId++;
  }
}

void LeggedRobotDummyVisualization::publishBaseTransform(ros::Time timeStamp, const SystemObservation& observation) const {
  // publish measured tf
  const Eigen::Vector3d p_base = observation.state.template segment<3>(6);
  const Eigen::Vector3d eulerAnglesWorldToBase = observation.state.template segment<3>(9);
  const Eigen::Quaterniond quaternionWorldToBase = getQuaternionFromEulerAnglesZyx(eulerAnglesWorldToBase);

  geometry_msgs::TransformStamped base_transform;
  base_transform.header.stamp = timeStamp;
  base_transform.header.frame_id = "odom";
  base_transform.child_frame_id = "base";
  base_transform.transform.translation.x = p_base.x();
  base_transform.transform.translation.y = p_base.y();
  base_transform.transform.translation.z = p_base.z();
  base_transform.transform.rotation.x = quaternionWorldToBase.x();
  base_transform.transform.rotation.y = quaternionWorldToBase.y();
  base_transform.transform.rotation.z = quaternionWorldToBase.z();
  base_transform.transform.rotation.w = quaternionWorldToBase.w();
  tfBroadcasterPtr_->sendTransform(base_transform);
}

void LeggedRobotDummyVisualization::publishOptimizedStateTrajectory(ros::Time timeStamp, const vector_array_t& mpcStateTrajectory) {
  const scalar_t TRAJECTORYLINEWIDTH = 0.005;
  const std::array<scalar_t, 3> red{0.6350, 0.0780, 0.1840};

  visualization_msgs::MarkerArray markerArray;
  visualization_msgs::Marker baseMarker;

  std::vector<geometry_msgs::Point> basePose;
  basePose.reserve(mpcStateTrajectory.size());

  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const state_vector_array_t::value_type& state) {
    const auto comPose = getComPose(state);
    basePose.push_back(getPointMsg(getPositionInOrigin(comPose)));
  });

  markerArray.markers.emplace_back(getLineMsg(std::move(basePose), red, TRAJECTORYLINEWIDTH));
  markerArray.markers.back().ns = "CoM Trajectory";

  assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg("odom", timeStamp));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

  stateOptimizedPublisher_.publish(markerArray);
}

}  // namespace legged_robot
}  // namespace ocs2
