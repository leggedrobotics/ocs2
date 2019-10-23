/*
 * MRT_ROS_Quadruped.h
 *
 *  Created on: May 25, 2018
 *      Author: farbod
 */

#ifndef MRT_ROS_QUADRUPED_H_
#define MRT_ROS_QUADRUPED_H_

#include <array>

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_core/control/ControllerType.h>

#include <ocs2_switched_model_interface/foot_planner/FeetZDirectionPlanner.h>
#include <ocs2_switched_model_interface/foot_planner/cpg/SplineCPG.h>
#include <ocs2_switched_model_interface/foot_planner/CubicSpline.h>
#include <ocs2_switched_model_interface/logic/SwitchedModelLogicRulesBase.h>

#include "ocs2_quadruped_interface/OCS2QuadrupedInterface.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM = 12 + JOINT_COORD_SIZE, size_t INPUT_DIM = 12 + JOINT_COORD_SIZE>
class MRT_ROS_Quadruped : public ocs2::MRT_ROS_Interface<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>>;

  using quadruped_interface_t = OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>;
  using quadruped_interface_ptr_t = typename quadruped_interface_t::Ptr;
  using contact_flag_t = typename quadruped_interface_t::contact_flag_t;
  using generalized_coordinate_t = typename quadruped_interface_t::generalized_coordinate_t;
  using joint_coordinate_t = typename quadruped_interface_t::joint_coordinate_t;
  using base_coordinate_t = typename quadruped_interface_t::base_coordinate_t;
  using rbd_state_vector_t = typename quadruped_interface_t::rbd_state_vector_t;

  enum { rbd_state_dim_ = quadruped_interface_t::rbd_state_dim_ };

  using BASE = ocs2::MRT_ROS_Interface<STATE_DIM, INPUT_DIM>;
  using typename BASE::command_data_t;
  using typename BASE::primal_solution_t;
  using typename BASE::system_observation_t;
  using typename BASE::controller_t;
  using typename BASE::scalar_t;
  using typename BASE::scalar_array_t;
  using typename BASE::size_array_t;
  using typename BASE::state_vector_t;
  using typename BASE::state_vector_array_t;
  using typename BASE::input_vector_t;
  using typename BASE::input_vector_array_t;
  using typename BASE::input_state_matrix_array_t;

  using feet_z_planner_t = FeetZDirectionPlanner<scalar_t, SplineCPG<scalar_t>>;
  using feet_z_planner_ptr_t = typename feet_z_planner_t::Ptr;

  // The base class for SplineCPG which is the return type of SwitchedModelPlannerLogicRules::getMotionPhaseLogics.
  using cpg_t = CPG_BASE<scalar_t>;
  using cpg_ptr_t = typename cpg_t::Ptr;
  using cubic_spline_t = CubicSpline<scalar_t>;
  using cubic_spline_ptr_t = typename cubic_spline_t::Ptr;

  using logic_rules_t = typename quadruped_interface_t::logic_rules_t;
  using logic_rules_ptr_t = typename logic_rules_t::Ptr;

  using vector_3d_t = Eigen::Matrix<scalar_t, 3, 1>;
  using vector_3d_array_t = std::array<vector_3d_t, 4>;

  /**
   * Constructor
   *
   * @param [in] ocs2QuadrupedInterfacePtr: A shared pointer to the quadruped interface class.
   * @param [in] robotName: The name's of the robot.
   */
  MRT_ROS_Quadruped(const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr, const std::string& robotName = "robot");

  /**
   * Destructor
   */
  virtual ~MRT_ROS_Quadruped() = default;

  using BASE::evaluatePolicy;
  using BASE::rolloutPolicy;

  /**
   * Computes the optimized plan for the given time based on the latest received optimized trajectory message.
   *
   * @param [in] time: inquiry time.
   * @param [in] state: inquiry state.
   * @param [out] o_feetPositionRef: Planned feet positions in the origin frame.
   * @param [out] o_feetVelocityRef: Planned feet velocities in the origin frame.
   * @param [out] o_feetAccelerationRef: Planned feet acceleration in the origin frame.
   * @param [out] o_comPoseRef: Planned CoM pose in the origin frame.
   * @param [out] o_comVelocityRef: Planned CoM velocity in the origin frame.
   * @param [out] o_comAccelerationRef: Planned CoM acceleration in the origin frame.
   * @param [out] stanceLegs: Planned stance legs.
   */
  void evaluatePolicy(const scalar_t& time, const state_vector_t& state, vector_3d_array_t& o_feetPositionRef,
                      vector_3d_array_t& o_feetVelocityRef, vector_3d_array_t& o_feetAccelerationRef, base_coordinate_t& o_comPoseRef,
                      base_coordinate_t& o_comVelocityRef, base_coordinate_t& o_comAccelerationRef, contact_flag_t& stanceLegs);

  /**
   * Computes the optimized plan for the given time based on the latest received optimized trajectory message.
   *
   * @param [in] time: inquiry time.
   * @param [in] state: inquiry state.
   * @param [out] o_feetPositionRef: Planned feet positions in the origin frame.
   * @param [out] o_feetVelocityRef: Planned feet velocities in the origin frame.
   * @param [out] o_feetAccelerationRef: Planned feet acceleration in the origin frame.
   * @param [out] o_comPoseRef: Planned CoM pose in the origin frame.
   * @param [out] o_comVelocityRef: Planned CoM velocity in the origin frame.
   * @param [out] o_comAccelerationRef: Planned CoM acceleration in the origin frame.
   * @param [out] stanceLegs: Planned stance legs.
   */
  void rolloutPolicy(const scalar_t& time, const state_vector_t& state, vector_3d_array_t& o_feetPositionRef,
                     vector_3d_array_t& o_feetVelocityRef, vector_3d_array_t& o_feetAccelerationRef, base_coordinate_t& o_comPoseRef,
                     base_coordinate_t& o_comVelocityRef, base_coordinate_t& o_comAccelerationRef, contact_flag_t& stanceLegs);

  /**
   * @brief Rolls out the control policy from the current time and state to get the next state and input using the MPC policy.
   *
   * @param [in] currentTime: start time of the rollout.
   * @param [in] currentState: state to start rollout from.
   * @param [in] timeStep: duration of the forward rollout.
   * @param [out] mpcState: the new forwarded state of MPC.
   * @param [out] mpcInput: the new control input of MPC.
   * @param [out] subsystem: the active subsystem.
   */
  void rolloutPolicy(scalar_t time, const rbd_state_vector_t& rbdState, rbd_state_vector_t& rbdStateRef, joint_coordinate_t& rbdInputRef,
                     size_t& subsystem);

  /**
   * Get the swing phase progress for a requested leg.
   *
   * @param [in] legIndex: Leg index.
   * @return swing phase progress
   */
  const scalar_t& getsSwingPhaseProgress(const size_t& legIndex) const;

  /**
   * Updates the publisher and subscriber nodes. Using the input measurements it constructs the observation
   * structure and publishes it using the base class method. It also checks calls the ros::spinOnce() and
   * checks for the policy update using the base class method.
   *
   * @param [in] contactFlag: Current contact flag.
   * @param [in] time: Current time.
   * @param [in] rbdState: Current robot's RBD state
   *
   * @return True if the policy is updated.
   */
  virtual bool updateNodes(const contact_flag_t& contactFlag, const scalar_t& time, const rbd_state_vector_t& rbdState);

 protected:
  /**
   * Finds the indices of a event times vector.
   *
   * @param eventTimes: Event time vector.
   * @param timeTrajectory: Control policy'r time stamp.
   * @param eventsIndices: event time indices over the control policy time stamp.
   */
  void findsIndicesEventTimes(const scalar_array_t& eventTimes, const scalar_array_t& timeTrajectory,
                              std::vector<int>& eventsIndices) const;

  void modifyPolicy(const command_data_t& command, primal_solution_t& primalSolution) override;

  void modifyBufferPolicy(const command_data_t& commandBuffer, primal_solution_t& primalSolutionBuffer) override;

  /**
   * Updates feet trajectories.
   *
   * @param [in] eventTimes: Event times.
   * @param [in] subsystemsSequence: Subsystems sequence.
   * @param [in] touchdownTimeStock: An array of touch-down times.
   * @param [in] touchdownStateStock: An array of feet positions at touch-down.
   * @param [in] touchdownInputStock: An array of feet velocities at touch-down.
   */
  virtual void updateFeetTrajectories(const scalar_array_t& eventTimes, const size_array_t& subsystemsSequence,
                                      const scalar_array_t& touchdownTimeStock, const state_vector_array_t& touchdownStateStock,
                                      const input_vector_array_t& touchdownInputStock);

  /**
   * Computes swing phase progress.
   *
   * @param [in] activeSubsystemIndex: Active subsystem index
   * @param [in] stanceLegs: Stance leg flags
   * @param [in] time: current time.
   * @param [out] swingPhaseProgress: swing phase progress for legs.
   */
  void computeSwingPhaseProgress(const size_t& activeSubsystemIndex, const contact_flag_t& stanceLegs, const scalar_t& time,
                                 std::array<scalar_t, 4>& swingPhaseProgress) const;

  /**
   * Computes feet's position, velocity, and acting contact force in the origin frame.
   *
   * @param [in] state: state vector.
   * @param [in] input: input vector.
   * @param [out] o_feetPosition: Feet's position in the origin frame.
   * @param [out] o_feetVelocity: Feet's velocity in the origin frame.
   * @param [out] o_contactForces: Feet's acting contact force in the origin frame.
   */
 public:
  virtual void computeFeetState(const state_vector_t& state, const input_vector_t& input, vector_3d_array_t& o_feetPosition,
                                vector_3d_array_t& o_feetVelocity, vector_3d_array_t& o_contactForces);

 private:
  /*
   * Variables
   */
  quadruped_interface_ptr_t ocs2QuadrupedInterfacePtr_;
  logic_rules_ptr_t logic_rules_mrt_;

  Model_Settings modelSettings_;

  std::array<const cpg_t*, 4> feetZPlanPtr_;
  std::array<scalar_t, 4> swingPhaseProgress_;

  std::vector<std::array<cubic_spline_ptr_t, 4>> feetXPlanPtrStock_;
  std::vector<std::array<cubic_spline_ptr_t, 4>> feetYPlanPtrStock_;

  scalar_array_t touchdownTimeStockBuffer_;
  state_vector_array_t touchdownStateStockBuffer_;
  input_vector_array_t touchdownInputStockBuffer_;
};

}  // end of namespace switched_model

#include "implementation/MRT_ROS_Quadruped.h"

#endif /* MRT_ROS_QUADRUPED_H_ */
