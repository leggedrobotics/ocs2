//
// Created by rgrandia on 13.02.19.
//

#ifndef QUADRUPEDXPPVISUALIZER_OCS2_H
#define QUADRUPEDXPPVISUALIZER_OCS2_H

#include <string>
#include <ros/ros.h>
#include <rosbag/bag.h>

#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_msgs/RobotStateCartesianTrajectory.h>
#include <xpp_msgs/RobotStateJoint.h>
#include <xpp_msgs/topic_names.h>

#include <ocs2_comm_interfaces/SystemObservation.h>
#include <ocs2_core/Dimensions.h>
#include <ocs2_quadruped_interface/OCS2QuadrupedInterface.h>
#include <ocs2_msgs/mpc_target_trajectories.h>
#include <ros/callback_queue.h>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>


namespace switched_model {

  namespace visualizers {

    /**
     * Ros Service to visualise the cost desired trajectories.
     */
    int triggerPublishingCostTrajectories(int argc, char** argv);
    inline const std::string costsPublishServiceName {"VisualiseCostDesiredTrajectories"};

  template <typename M>
    struct PublisherMapping{
      typedef M msg_t;
      const std::string topicName_;
      ros::Publisher advertise(ros::NodeHandle& n, size_t rate, bool latch) const {return n.advertise<msg_t>(topicName_, rate, latch);};
      ros::Publisher advertise(ros::NodeHandle& n, size_t rate) const
      {
        return n.advertise<msg_t>(topicName_, rate);
      };
      PublisherMapping(const std::string topicName): topicName_(topicName){};
    };

  /***************************************************************************************************/
  inline static const std::string xppJointDesTopicName = "xpp/joint_anymal_des";
  inline static const std::string xppStateDesTopicName = "/xpp/state_des"; //xpp_msgs::robot_state_desired;

  const static auto xppStateDesTopicMap = PublisherMapping<xpp_msgs::RobotStateCartesian>(xppStateDesTopicName);
  const static auto xppJointDesTopicMap = PublisherMapping<xpp_msgs::RobotStateJoint>(xppJointDesTopicName);
  /***************************************************************************************************/
  inline static const std::string xppStateTrajTopicName = xppStateDesTopicName.substr(0, xppStateDesTopicName.find("des")) + "traj";
  inline static const std::string xppJointTrajTopicName = xppJointDesTopicName.substr(0, xppJointDesTopicName.find("des")) + "traj";

  static const auto xppStateTrajTopicMap = PublisherMapping<xpp_msgs::RobotStateCartesian>(xppStateTrajTopicName);
  static const auto xppJointTrajTopicMap = PublisherMapping<xpp_msgs::RobotStateJoint>(xppJointTrajTopicName);
  /***************************************************************************************************/

  inline static const std::string comTraceTopicName     = "baseTrajectory"; //desiredBaseTrajectory
  inline static const std::string feetTraceTopicName    = "feetTrajectories";
  inline static const std::string posesTargetTopicName  = "poseTargets";
  const auto comTraceTopicMap     = PublisherMapping<visualization_msgs::Marker>(comTraceTopicName);
  const auto feetTraceTopicMap    = PublisherMapping<visualization_msgs::MarkerArray>(feetTraceTopicName);
  const auto posesTargetTopicMap  = PublisherMapping<geometry_msgs::PoseArray>(posesTargetTopicName);

  inline static const std::string comMPCTraceTopicName  = "baseMPCTrajectory"; //desiredBaseTrajectory
  inline static const std::string feetMPCTraceTopicName = "feetMPCTrajectories";
  inline static const std::string posesMPCTopicName     = "poseMPC";
  const static auto comMPCTraceTopicMap  = PublisherMapping<visualization_msgs::Marker>(comMPCTraceTopicName);
  const static auto feetMPCTraceTopicMap = PublisherMapping<visualization_msgs::MarkerArray>(feetMPCTraceTopicName);
  const static auto posesMPCTopicMap     = PublisherMapping<geometry_msgs::PoseArray>(posesMPCTopicName);
  }

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
class QuadrupedXppVisualizer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


  using quadruped_xpp_visualizer_t = QuadrupedXppVisualizer<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>;
  using quadruped_interface_t = OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>;
  using quadruped_interface_ptr_t = typename quadruped_interface_t::Ptr;
  using rbd_state_vector_t = typename quadruped_interface_t::rbd_state_vector_t;
  using scalar_t = typename quadruped_interface_t::scalar_t;
  using state_vector_t = typename quadruped_interface_t::state_vector_t;
  using scalar_array_t = typename quadruped_interface_t::scalar_array_t;
  using state_vector_array_t = typename quadruped_interface_t::state_vector_array_t;
  using input_vector_t = typename quadruped_interface_t::input_vector_t;
  using cost_desired_trajectories_t = typename quadruped_interface_t::cost_desired_trajectories_t;

  using system_observation_t = ocs2::SystemObservation<STATE_DIM, INPUT_DIM>;
  using system_observation_array_t = std::vector<system_observation_t, Eigen::aligned_allocator<system_observation_t>>;
  using vector_3d_t = Eigen::Matrix<scalar_t, 3, 1>;
  using vector_3d_array_t = std::array<vector_3d_t, 4>;


  QuadrupedXppVisualizer(const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr, const std::string& robotName = "robot",
                         bool save_rosbag = false)
      : ocs2QuadrupedInterfacePtr_(ocs2QuadrupedInterfacePtr), robotName_(robotName), save_rosbag_(save_rosbag) {
    if (save_rosbag_) {
      rosbagFile_ = "ocs2_" + robotName_ + "_traj.bag";
      bag_.open(rosbagFile_, rosbag::bagmode::Write);
    }
  };

  ~QuadrupedXppVisualizer() {
    if (save_rosbag_) {
      ROS_INFO_STREAM("ROS Bag file is being saved to \"" + rosbagFile_ + "\" ...");
      try {
        bag_.write("xpp/trajectory_des", startTime_, robotStateCartesianTrajectoryMsg_);
      } catch (const rosbag::BagException& err) {
        std::cerr << "Error writing rosbag message: " << err.what() << std::endl;
      }
      bag_.close();
      ROS_INFO_STREAM("ROS Bag file is saved.");
    }
  }

  /**
   * Launches the visualization node
   *
   * @param [in] argc: command line number of inputs.
   * @param [in] argv: command line inputs' value.
   */
  void launchVisualizerNode(int argc, char* argv[]);


  /**
   * Visualizes the current observation.
   *
   * @param [in] observation: The current observation.
   */
  void publishObservation(const system_observation_t& observation);

  /**
   * Visualize trajectory of observations.
   *
   * @param [in] observation_array: Array of observations.
   */
  void publishTrajectory(const system_observation_array_t& system_observation_array, double speed = 1.0);

  void publishDesiredTrajectory(scalar_t startTime, const cost_desired_trajectories_t& costDesiredTrajectory);

  void publishOptimizedStateTrajectory(const scalar_array_t& mpcTimeTrajectory, const state_vector_array_t& mpcStateTrajectory);

  void publishXppCostsVisualizer(const scalar_t& publishTime, const cost_desired_trajectories_t& costDesiredTrajectory);
  void callVisualizerQueue() { visualizerQueue_.callAvailable(); };

  // void publishDesiredTrajectory(scalar_t startTime);
  // void publishXppCostsVisualizer( const scalar_t& publishTime);

 private:
  /**
   * Publishes the xpp visualization messages and also if "SAVE_ROS_BAG" is defined, it saves then in a ROS bag file.
   *
   * @param time: time.
   * @param basePose: Base pose in the origin frame.
   * @param baseLocalVelocities: Base local velocities.
   * @param feetPosition: Feet position in the origin frame.
   * @param feetVelocity: Feet velocity in the origin frame.
   * @param feetAcceleration: Feet acceleration in the origin frame.
   * @param feetForce: Contact forces acting on the feet in the origin frame.
   */
  void publishXppVisualizer(const scalar_t time, const base_coordinate_t& basePose, const base_coordinate_t& baseLocalVelocities,
                            const joint_coordinate_t& jointAngles,
                            const vector_3d_array_t& feetPosition, const vector_3d_array_t& feetVelocity,
                            const vector_3d_array_t& feetAcceleration, const vector_3d_array_t& feetForce);

  void computeFeetState(const state_vector_t& state, const input_vector_t& input, vector_3d_array_t& o_feetPosition,
                        vector_3d_array_t& o_feetVelocity, vector_3d_array_t& o_contactForces);

  static void sigintHandler(int sig);

  /**
   * Ros callback to visualise the mpc target trajectories.
   */
  void targetTrajectoriesCallback(const ocs2_msgs::mpc_target_trajectories &);

  quadruped_interface_ptr_t ocs2QuadrupedInterfacePtr_;

  bool save_rosbag_;
  std::string robotName_;
  std::string rosbagFile_;

  ros::ServiceServer visualsCDTsClient_;
  ros::Publisher visualizationPublisher_;
  ros::Publisher visualizationJointPublisher_;

  ros::Publisher costDesiredPublisher_;

  ros::Publisher comTracePublisher_;
  ros::Publisher feetTracePublisher_;
  ros::Publisher poseTrajPublisher_;

  ros::Publisher comMPCTracePublisher_;
  ros::Publisher feetMPCTracePublisher_;
  ros::Publisher poseMPCPublisher_;

  ros::Publisher costsVisualizationPublisher_;
  ros::Publisher costsVisualizationJointPublisher_;

  ros::Time startTime_;
  ros::Subscriber targetTrajectoriesSubscriber_;
  ros::CallbackQueue visualizerQueue_;

  rosbag::Bag bag_;
  xpp_msgs::RobotStateCartesianTrajectory robotStateCartesianTrajectoryMsg_;

  /**************************************************************************************************/
    inline void setRobotStateCartesianEEValues( const state_vector_t& state, const input_vector_t& input, xpp_msgs::RobotStateCartesian& rstcm);
    inline void setRobotStateCartesianEEValues( const vector_3d_array_t& o_feetPosition,
        const vector_3d_array_t& o_feetVelocity,
        const vector_3d_array_t& o_feetAcceleration,
        const vector_3d_array_t& o_feetForce,
        xpp_msgs::RobotStateCartesian& rstcm);
};

}  // namespace switched_model

#include "implementation/QuadrupedXppVisualizer.h"

#endif  // OCS2_QUADRUPEDXPPVISUALIZER_OCS2_H
