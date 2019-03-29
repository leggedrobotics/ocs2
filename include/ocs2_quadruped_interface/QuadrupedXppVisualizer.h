//
// Created by rgrandia on 13.02.19.
//

#ifndef QUADRUPEDXPPVISUALIZER_OCS2_H
#define QUADRUPEDXPPVISUALIZER_OCS2_H

#include <rosbag/bag.h>

#include <xpp_msgs/RobotStateCartesianTrajectory.h>
#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_msgs/topic_names.h>

#include <ocs2_core/Dimensions.h>
#include <ocs2_comm_interfaces/SystemObservation.h>
#include <ocs2_quadruped_interface/OCS2QuadrupedInterface.h>

namespace switched_model {

    template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
    class QuadrupedXppVisualizer {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM> quadruped_interface_t;
        typedef typename quadruped_interface_t::Ptr quadruped_interface_ptr_t;

        typedef typename quadruped_interface_t::contact_flag_t			contact_flag_t;
        typedef typename quadruped_interface_t::generalized_coordinate_t generalized_coordinate_t;
        typedef typename quadruped_interface_t::joint_coordinate_t 		joint_coordinate_t;
        typedef typename quadruped_interface_t::base_coordinate_t 		base_coordinate_t;
        typedef typename quadruped_interface_t::rbd_state_vector_t		rbd_state_vector_t;

        typedef ocs2::Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
        typedef typename DIMENSIONS::scalar_t   scalar_t;
        typedef typename DIMENSIONS::state_vector_t   state_vector_t;
        typedef typename DIMENSIONS::input_vector_t   input_vector_t;
        typedef ocs2::SystemObservation<STATE_DIM, INPUT_DIM> system_observation_t;
        typedef std::vector<system_observation_t, Eigen::aligned_allocator<system_observation_t>> system_observation_array_t;

        typedef Eigen::Matrix<scalar_t, 3, 1>	vector_3d_t;
        typedef std::array<vector_3d_t, 4>	vector_3d_array_t;

        QuadrupedXppVisualizer(const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr,
                               const std::string& robotName = "robot",
                                bool save_rosbag = false) :
                ocs2QuadrupedInterfacePtr_(ocs2QuadrupedInterfacePtr),
                robotName_(robotName),
                save_rosbag_(save_rosbag) {
            if (save_rosbag_) {
                rosbagFile_ = "ocs2_" + robotName_ + "_traj.bag";
                bag_.open(rosbagFile_, rosbag::bagmode::Write);
            }
        };

        ~QuadrupedXppVisualizer() {
            if (save_rosbag_){
                ROS_INFO_STREAM("ROS Bag file is being saved to \"" + rosbagFile_ + "\" ...");
                bag_.write("xpp/trajectory_des", startTime_, robotStateCartesianTrajectoryMsg_);
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
        void publishXppVisualizer(
                const scalar_t& time,
                const base_coordinate_t& basePose,
                const base_coordinate_t& baseLocalVelocities,
                const vector_3d_array_t& feetPosition,
                const vector_3d_array_t& feetVelocity,
                const vector_3d_array_t& feetAcceleration,
                const vector_3d_array_t& feetForce);

        void computeFeetState(
                const state_vector_t& state,
                const input_vector_t& input,
                vector_3d_array_t& o_feetPosition,
                vector_3d_array_t& o_feetVelocity,
                vector_3d_array_t& o_contactForces);

        static void sigintHandler(int sig);

        quadruped_interface_ptr_t ocs2QuadrupedInterfacePtr_;

        bool save_rosbag_;
        std::string robotName_;
        std::string rosbagFile_;

        ros::Publisher visualizationPublisher_;
        ros::Time startTime_;

        rosbag::Bag bag_;
        xpp_msgs::RobotStateCartesianTrajectory robotStateCartesianTrajectoryMsg_;
    };

}

#include "implementation/QuadrupedXppVisualizer.h"


#endif //OCS2_QUADRUPEDXPPVISUALIZER_OCS2_H
