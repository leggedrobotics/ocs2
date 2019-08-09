/*
 * MRT_ROS_Dummy_Quadruped.h
 *
 *  Created on: May 28, 2018
 *      Author: farbod
 */

#ifndef MRT_ROS_DUMMY_QUADRUPED_H_
#define MRT_ROS_DUMMY_QUADRUPED_H_

#include <unistd.h>

#include <rosbag/bag.h>

#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_msgs/RobotStateCartesianTrajectory.h>
#include <xpp_msgs/topic_names.h>

#include <ocs2_comm_interfaces/test/MRT_ROS_Dummy_Loop.h>
#include "ocs2_quadruped_interface/MRT_ROS_Quadruped.h"
#include "ocs2_quadruped_interface/QuadrupedXppVisualizer.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM = 12 + JOINT_COORD_SIZE, size_t INPUT_DIM = 12 + JOINT_COORD_SIZE>
class MRT_ROS_Dummy_Quadruped : public ocs2::MRT_ROS_Dummy_Loop<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using quadruped_interface_t = OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>;
  using quadruped_interface_ptr_t = typename quadruped_interface_t::Ptr;

  using contact_flag_t = typename quadruped_interface_t::contact_flag_t;
  using generalized_coordinate_t = typename quadruped_interface_t::generalized_coordinate_t;
  using joint_coordinate_t = typename quadruped_interface_t::joint_coordinate_t;
  using base_coordinate_t = typename quadruped_interface_t::base_coordinate_t;
  using rbd_state_vector_t = typename quadruped_interface_t::rbd_state_vector_t;

  enum { rbd_state_dim_ = quadruped_interface_t::rbd_state_dim_ };

  using BASE = ocs2::MRT_ROS_Dummy_Loop<STATE_DIM, INPUT_DIM>;
  using typename BASE::commandData_t;
  using typename BASE::cost_desired_trajectories_t;
  using typename BASE::input_state_matrix_array_t;
  using typename BASE::input_state_matrix_t;
  using typename BASE::input_vector_array_t;
  using typename BASE::input_vector_t;
  using typename BASE::policyData_t;
  using typename BASE::scalar_array_t;
  using typename BASE::scalar_t;
  using typename BASE::size_array_t;
  using typename BASE::state_vector_array_t;
  using typename BASE::state_vector_t;
  using typename BASE::system_observation_t;

  using vector_3d_t = Eigen::Matrix<scalar_t, 3, 1>;
  using vector_3d_array_t = std::array<vector_3d_t, 4>;

  using mrt_t = MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>;
  using mrt_ptr_t = typename mrt_t::Ptr;

  using visualizer_t = QuadrupedXppVisualizer<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>;

  /**
   * Constructor.
   *
   * @param [in] ocs2QuadrupedInterfacePtr
   * @param [in] mrtDesiredFrequency: MRT loop frequency in Hz
   * @param [in] robotName
   * @param [in] mpcDesiredFrequency: MPC loop frequency in Hz
   */
  MRT_ROS_Dummy_Quadruped(quadruped_interface_ptr_t ocs2QuadrupedInterfacePtr, scalar_t mrtDesiredFrequency,
                          std::string robotName = "robot", scalar_t mpcDesiredFrequency = -1);

  /**
   * Constructor.
   *
   * @param [in] ocs2QuadrupedInterfacePtr
   * @param [in] mrtPtr: A pointer to MRT instance
   * @param [in] mrtDesiredFrequency: MRT loop frequency in Hz
   * @param [in] robotName
   * @param [in] mpcDesiredFrequency: MPC loop frequency in Hz
   */
  MRT_ROS_Dummy_Quadruped(quadruped_interface_ptr_t ocs2QuadrupedInterfacePtr, mrt_ptr_t mrtPtr, scalar_t mrtDesiredFrequency,
                          std::string robotName = "robot", scalar_t mpcDesiredFrequency = -1);

  /**
   * Default destructor.
   */
  virtual ~MRT_ROS_Dummy_Quadruped() = default;

 protected:
  void modifyObservation(system_observation_t& observation) override {}

  void launchVisualizerNode(int argc, char* argv[]) override;

  void publishVisualizer(const system_observation_t& observation, const commandData_t& command, const policyData_t& policy) override;

 private:
  visualizer_t quadrupedXppVisualizer_;
};

}  // end of namespace switched_model

#include "implementation/MRT_ROS_Dummy_Quadruped.h"

#endif /* MRT_ROS_DUMMY_QUADRUPED_H_ */
