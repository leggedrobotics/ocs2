//
// Created by rgrandia on 25.03.22.
//

#include <ros/init.h>
#include <ros/node_handle.h>

// Additional messages not in the helpers file
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <ocs2_robotic_assets/package_path.h>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

#include <ocs2_quadruped_interface/QuadrupedTfPublisher.h>

#include "ocs2_anymal_motion_tracking/CartesianMotionTrackingCost.h"
#include "ocs2_anymal_motion_tracking/ModelParentSkeleton.h"
#include "ocs2_anymal_motion_tracking/MotionTracking.h"

namespace {
const std::string URDF_FILE = ocs2::robotic_assets::getPath() + "/resources/anymal_c/urdf/anymal.urdf";
}  // unnamed namespace

namespace switched_model {

void toPoseMsg(const Pose<scalar_t>& pose, geometry_msgs::Pose& poseMsg) {
  poseMsg.position = ocs2::getPointMsg(pose.position);
  poseMsg.orientation = ocs2::getOrientationMsg(Eigen::Quaterniond(pose.orientation));
}

template <typename SCALAR_T>
void simulate(const Twist<SCALAR_T>& twist, Pose<SCALAR_T>& pose, SCALAR_T dt) {
  pose.position += dt * twist.linear;
  const auto angNorm = twist.angular.norm();
  if (angNorm > SCALAR_T(1e-12)) {
    pose.orientation = pose.orientation * Eigen::AngleAxis<SCALAR_T>(dt * angNorm, twist.angular / angNorm);
  }
}

}  // namespace switched_model

using namespace switched_model;

class MotionTargetPublisher {
 public:
  MotionTargetPublisher(ros::NodeHandle& nodeHandle, const std::string& poseTopic, const std::string& velocityTopic) {
    posePublisher_ = nodeHandle.advertise<geometry_msgs::PoseArray>(poseTopic, 1);
    velocityPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>(velocityTopic, 1);
  }

  void publish(const std::vector<MotionTarget<scalar_t>>& targets, const std::string& frame, const ros::Time& time) {
    poseMsg_.poses.resize(targets.size());
    for (int i = 0; i < targets.size(); ++i) {
      toPoseMsg(targets[i].poseInWorld, poseMsg_.poses[i]);
    }
    poseMsg_.header = ocs2::getHeaderMsg(frame, time);

    velocityMsg_.markers.resize(2 * targets.size());
    scalar_t scale = 0.1;
    for (int i = 0; i < targets.size(); ++i) {
      velocityMsg_.markers[2 * i] =
          ocs2::getArrowAtPointMsg(scale * targets[i].twistInWorld.linear, targets[i].poseInWorld.position, ocs2::Color::blue);
      velocityMsg_.markers[2 * i + 1] =
          ocs2::getArrowAtPointMsg(scale * targets[i].twistInWorld.angular, targets[i].poseInWorld.position, ocs2::Color::green);
    }

    // Give markers an id and a frame
    ocs2::assignHeader(velocityMsg_.markers.begin(), velocityMsg_.markers.end(), poseMsg_.header);
    ocs2::assignIncreasingId(velocityMsg_.markers.begin(), velocityMsg_.markers.end());

    posePublisher_.publish(poseMsg_);
    velocityPublisher_.publish(velocityMsg_);
  }

 private:
  ros::Publisher posePublisher_;
  ros::Publisher velocityPublisher_;
  geometry_msgs::PoseArray poseMsg_;
  visualization_msgs::MarkerArray velocityMsg_;
};

class MotionTaskPublisher {
 public:
  MotionTaskPublisher(ros::NodeHandle& nodeHandle, const std::string& topic) {
    publisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>(topic, 1);
  }

  void publish(const std::vector<AbsoluteTrackingTask<scalar_t>>& tasks, const std::vector<MotionTarget<scalar_t>>& sourceFrames,
               const std::vector<MotionTarget<scalar_t>>& targetFrames, const std::string& frame, const ros::Time& time) {
    int markersPerTask = 2;
    msg_.markers.resize(markersPerTask * tasks.size());

    scalar_t scale = 0.01;
    for (int i = 0; i < tasks.size(); ++i) {
      const auto& task = tasks[i];
      const auto& source = sourceFrames[task.sourceId];
      const auto& target = targetFrames[task.targetId];

      msg_.markers[markersPerTask * i] = ocs2::getLineMsg(
          {ocs2::getPointMsg(source.poseInWorld.position), ocs2::getPointMsg(target.poseInWorld.position)}, ocs2::Color::black, scale);

      auto costGradient = -getCostVector(task, sourceFrames, targetFrames);
      if (task.type == TrackingTaskType::POSITION) {
        msg_.markers[markersPerTask * i + 1] = ocs2::getArrowAtPointMsg(costGradient, source.poseInWorld.position, ocs2::Color::orange);
      } else {
        msg_.markers[markersPerTask * i + 1] = ocs2::getArrowAtPointMsg(costGradient, source.poseInWorld.position, ocs2::Color::purple);
      }
    }

    // Give markers an id and a frame
    ocs2::assignHeader(msg_.markers.begin(), msg_.markers.end(), ocs2::getHeaderMsg(frame, time));
    ocs2::assignIncreasingId(msg_.markers.begin(), msg_.markers.end());

    publisher_.publish(msg_);
  }

 private:
  ros::Publisher publisher_;
  visualization_msgs::MarkerArray msg_;
};

int main(int argc, char* argv[]) {
  // Initialize ros node
  ros::init(argc, argv, "anymal_motion_tracking_target_viz");
  ros::NodeHandle nodeHandle;
  //
  //  std::string urdfFile = "";
  //  nodeHandle.getParam("/urdfFile", urdfFile);

  ModelParentSkeleton<scalar_t> skeleton(getPinocchioInterface<scalar_t>(URDF_FILE), {"base", "LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"});
  ModelParentSkeleton<ad_scalar_t> skeletonAd(getPinocchioInterface<ad_scalar_t>(URDF_FILE),
                                              {"base", "LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"});

  auto parentPublisher_ = MotionTargetPublisher(nodeHandle, "/ocs2_anymal/tracking_parents", "/ocs2_anymal/tracking_parents_vel");
  auto sourcesPublisher_ = MotionTargetPublisher(nodeHandle, "/ocs2_anymal/tracking_sources", "/ocs2_anymal/tracking_sources_vel");
  auto targetsPublisher_ = MotionTargetPublisher(nodeHandle, "/ocs2_anymal/tracking_targets", "/ocs2_anymal/tracking_targets_vel");
  auto costPublisher_ = MotionTaskPublisher(nodeHandle, "/ocs2_anymal/tracking_costs");

  //  std::vector<MotionTarget<ocs2::scalar_t>> parents;
  //  parents.push_back(MotionTarget<ocs2::scalar_t>::Zero());
  //  parents.push_back(MotionTarget<ocs2::scalar_t>::Random());
  //  parents.front().twistInWorld.angular = vector3_t{0.0, 0.0, 1.0};

  vector_t x = state_vector_t::Zero();
  vector_t u = input_vector_t::Zero();
  std::vector<MotionTarget<scalar_t>> parents = skeleton.update(x, u);

  vector_t xTarget = state_vector_t::Zero();
  xTarget.head(3) = 3.0 * vector3_t::Random();
  xTarget.tail(JOINT_COORDINATE_SIZE) = 0.5 * joint_coordinate_t::Random();
  vector_t uTarget = input_vector_t::Zero();
  std::vector<MotionTarget<scalar_t>> targets = skeleton.update(xTarget, uTarget);

  std::vector<AbsoluteTrackingTask<scalar_t>> tasks;
  std::vector<TrackingOffset<scalar_t>> offsets;
  std::vector<MotionTarget<ocs2::scalar_t>> sources(parents.size());
  for (int i = 0; i < parents.size(); ++i) {
    offsets.push_back({i, Pose<scalar_t>::Zero()});
    AbsoluteTrackingTask<scalar_t> task;
    task.type = TrackingTaskType::POSITION;
    task.targetId = i;
    task.sourceId = i;
    task.poseWeights.setOnes();
    tasks.push_back(task);
  }
  AbsoluteTrackingTask<scalar_t> task;
  task.type = TrackingTaskType::ORIENTATION;
  task.targetId = 0;
  task.sourceId = 0;
  task.poseWeights.setOnes();
  tasks.push_back(task);

  CartesianMotionTrackingCost costFunction(skeletonAd, tasks, offsets, targets, true);

  QuadrupedTfPublisher tfPublisher;
  tfPublisher.launchNode(nodeHandle, "ocs2_anymal_description");

  scalar_t dt = 1.0 / 1.0;
  ros::Rate rate(1.0 / dt);
  while (ros::ok()) {
    // Take a step
    costFunction.updateTargets(targets);
    const auto qpApprox = costFunction.getQuadraticApproximation(0.0, x, u, ocs2::TargetTrajectories(), ocs2::PreComputation());
    vector_t dx = -qpApprox.dfdxx.ldlt().solve(qpApprox.dfdx);

    // linesearch
    scalar_t alpha = 1.0;
    while (alpha > 1e-6) {
      scalar_t f = costFunction.getValue(0.0, x + alpha * dx, u, ocs2::TargetTrajectories(), ocs2::PreComputation());
      if (f < qpApprox.f) {
        x += alpha * dx;
        break;
      } else {
        alpha *= 0.5;
      }
    }

    // Compute costs
    const int costsPerTask = 3;
    ocs2::vector_t y(costsPerTask * tasks.size());
    for (int i = 0; i < tasks.size(); ++i) {
      y.segment(costsPerTask * i, 3) = getCostVector(tasks[i], sources, targets);
    }

    //    std::cout << "x: " << x.transpose() << std::endl;
    //    std::cout << "xTarget: " << xTarget.transpose() << std::endl;
    //    std::cout << "dfdx: " << qpApprox.dfdx.transpose() << std::endl;
    //    std::cout << "dx: " << dx.transpose() << std::endl;
    //    std::cout << "y: " << y.transpose() << std::endl;
    //    std::cout << "=======================================================================" << std::endl;

    // Update frames
    parents = skeleton.update(x, u);
    updateSourceFrames(parents, offsets, sources);

    // Publish
    auto time = ros::Time::now();
    parentPublisher_.publish(parents, "world", time);
    sourcesPublisher_.publish(sources, "world", time);
    targetsPublisher_.publish(targets, "world", time);
    costPublisher_.publish(tasks, sources, targets, "world", time);
    tfPublisher.publish(time, x, "world");

    // Change target
    if (y.norm() < 1e-6) {
      xTarget.head(3) = 3.0 * vector3_t::Random();
      xTarget.tail(JOINT_COORDINATE_SIZE) = 0.5 * joint_coordinate_t::Random();
      targets = skeleton.update(xTarget, uTarget);
    }

    rate.sleep();
  }
}