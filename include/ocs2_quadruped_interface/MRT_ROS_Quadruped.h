/*
 * MRT_ROS_Quadruped.h
 *
 *  Created on: May 25, 2018
 *      Author: farbod
 */

#ifndef MRT_ROS_QUADRUPED_H_
#define MRT_ROS_QUADRUPED_H_

#include <array>

#include <ocs2_core/control/ControllerType.h>
#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <ocs2_switched_model_interface/misc/CubicSpline.h>
#include <ocs2_switched_model_interface/foot_planner/cpg/SplineCPG.h>
#include <ocs2_switched_model_interface/foot_planner/FeetZDirectionPlanner.h>
#include <ocs2_switched_model_interface/logic/SwitchedModelLogicRulesBase.h>

#include "ocs2_quadruped_interface/OCS2QuadrupedInterface.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM=12+JOINT_COORD_SIZE, size_t INPUT_DIM=12+JOINT_COORD_SIZE>
class MRT_ROS_Quadruped : public ocs2::MRT_ROS_Interface<STATE_DIM, INPUT_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>> Ptr;

	typedef OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM> quadruped_interface_t;
	typedef typename quadruped_interface_t::Ptr 					quadruped_interface_ptr_t;
	typedef typename quadruped_interface_t::contact_flag_t			contact_flag_t;
	typedef typename quadruped_interface_t::generalized_coordinate_t generalized_coordinate_t;
	typedef typename quadruped_interface_t::joint_coordinate_t 		joint_coordinate_t;
	typedef typename quadruped_interface_t::base_coordinate_t 		base_coordinate_t;
	typedef typename quadruped_interface_t::rbd_state_vector_t		rbd_state_vector_t;

	enum {
		rbd_state_dim_ = quadruped_interface_t::rbd_state_dim_
	};

	typedef ocs2::MRT_ROS_Interface<STATE_DIM, INPUT_DIM> BASE;
	typedef typename BASE::system_observation_t 	system_observation_t;
	typedef typename BASE::scalar_t					scalar_t;
	typedef typename BASE::scalar_array_t			scalar_array_t;
	typedef typename BASE::size_array_t				size_array_t;
	typedef typename BASE::state_vector_t			state_vector_t;
	typedef typename BASE::state_vector_array_t		state_vector_array_t;
	typedef typename BASE::input_vector_t			input_vector_t;
	typedef typename BASE::input_vector_array_t		input_vector_array_t;
	typedef typename BASE::input_state_matrix_t		input_state_matrix_t;
	typedef typename BASE::input_state_matrix_array_t input_state_matrix_array_t;

	typedef typename BASE::controller_t controller_t;

	typedef FeetZDirectionPlanner<scalar_t,SplineCPG<scalar_t>>	feet_z_planner_t;
	typedef typename feet_z_planner_t::Ptr						feet_z_planner_ptr_t;

	// The base class for SplineCPG which is the return type of SwitchedModelPlannerLogicRules::getMotionPhaseLogics.
	typedef CPG_BASE<scalar_t>				cpg_t;
	typedef typename cpg_t::Ptr				cpg_ptr_t;
	typedef CubicSpline<scalar_t>			cubic_spline_t;
	typedef typename cubic_spline_t::Ptr	cubic_spline_ptr_t;

	typedef typename quadruped_interface_t::logic_rules_t 	logic_rules_t;
	typedef typename logic_rules_t::Ptr 					logic_rules_ptr_t;

	typedef Eigen::Matrix<scalar_t,3,1>	vector_3d_t;
	typedef std::array<vector_3d_t,4>	vector_3d_array_t;

	/**
	 * Constructor
	 *
	 * @param [in] ocs2QuadrupedInterfacePtr: A shared pointer to the quadruped interface class.
	 * @param [in] robotName: The name's of the robot.
	 */
	MRT_ROS_Quadruped(
			const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr,
			const std::string& robotName = "robot");

	/**
	 * Destructor
	 */
	virtual ~MRT_ROS_Quadruped() = default;

	using BASE::evaluatePolicy;

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
	void evaluatePolicy(
			const scalar_t& time,
			const state_vector_t& state,
			vector_3d_array_t& o_feetPositionRef,
			vector_3d_array_t& o_feetVelocityRef,
			vector_3d_array_t& o_feetAccelerationRef,
			base_coordinate_t& o_comPoseRef,
			base_coordinate_t& o_comVelocityRef,
			base_coordinate_t& o_comAccelerationRef,
			contact_flag_t& stanceLegs);

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
	void rolloutPolicy(
			const scalar_t& time,
			const state_vector_t& state,
			vector_3d_array_t& o_feetPositionRef,
			vector_3d_array_t& o_feetVelocityRef,
			vector_3d_array_t& o_feetAccelerationRef,
			base_coordinate_t& o_comPoseRef,
			base_coordinate_t& o_comVelocityRef,
			base_coordinate_t& o_comAccelerationRef,
			contact_flag_t& stanceLegs);

//	/**
//	 * Computes the optimized plan for the given time based on the latest received optimized trajectory message.
//	 *
//	 * @param [in] time: inquiry time.
//	 * @param [out] o_feetPositionRef: Planned feet positions in the origin frame.
//	 * @param [out] o_feetVelocityRef: Planned feet velocities in the origin frame.
//	 * @param [out] o_feetAccelerationRef: Planned feet acceleration in the origin frame.
//	 * @param [out] comPoseRef: Planned CoM pose in the origin frame.
//	 * @param [out] comVelocityRef: Planned CoM velocity in the origin frame.
//	 * @param [out] comAccelerationRef: Planned CoM acceleration in the origin frame.
//	 * @param [out] stanceLegs: Planned stance legs.
//	 */
//	void computePlan(
//			const scalar_t& time,
//			vector_3d_array_t& o_feetPositionRef,
//			vector_3d_array_t& o_feetVelocityRef,
//			vector_3d_array_t& o_feetAccelerationRef,
//			base_coordinate_t& o_comPoseRef,
//			base_coordinate_t& o_comVelocityRef,
//			base_coordinate_t& o_comAccelerationRef,
//			contact_flag_t& stanceLegs);

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
	virtual bool updateNodes(
			const contact_flag_t& contactFlag,
			const scalar_t& time,
			const rbd_state_vector_t& rbdState);

protected:
	/**
	 * Finds the indices of a event times vector.
	 *
	 * @param eventTimes: Event time vector.
	 * @param timeTrajectory: Control policy'r time stamp.
	 * @param eventsIndices: event time indices over the control policy time stamp.
	 */
	void findsIndicesEventTimes(
			const scalar_array_t& eventTimes,
			const scalar_array_t& timeTrajectory,
			std::vector<int>& eventsIndices) const;

	/**
	 * This method can be used to modify the policy on the buffer without inputting the
	 * main thread. Note that the variables that are on the buffer have the suffix Buffer. It is
	 * important if any new variables are added to the policy also obey this rule. These buffer
	 * variables can be later, in the modifyPolicy() method, swept to the in-use policy memory.
	 *
	 * @param [in] mpcInitObservationBuffer: The observation of the policy message on the buffer.
	 * @param mpcControllerBuffer: The optimized controller of the policy message on the buffer.
	 * @param mpcTimeTrajectoryBuffer: The optimized time trajectory of the policy message on the buffer.
	 * @param mpcStateTrajectoryBuffer: The optimized state trajectory of the policy message on the buffer.
	 * @param eventTimesBuffer: The event times of the policy message on the buffer.
	 * @param subsystemsSequenceBuffer: The subsystems sequence of the policy message on the buffer.
	 */
	virtual void modifyBufferPolicy(
			const system_observation_t& mpcInitObservationBuffer,
			controller_t& mpcControllerBuffer,
			scalar_array_t& mpcTimeTrajectoryBuffer,
			state_vector_array_t& mpcStateTrajectoryBuffer,
			scalar_array_t& eventTimesBuffer,
			size_array_t& subsystemsSequenceBuffer) override;

	/**
	 * The updatePolicy() method will call this method which allows the user to
	 * customize the in-use policy. Note that this method is already
	 * protected with a mutex which blocks the policy callback. Moreover, this method
	 * may be called in the main thread of the program. Thus, for efficiency and
	 * practical considerations you should avoid computationally expensive operations.
	 * For such operations you may want to use the modifyBufferPolicy()
	 * methods which runs on a separate thread which directly modifies the received
	 * policy messages on the data buffer.
	 *
	 * @param logicUpdated: Whether eventTimes or subsystemsSequence are updated form the last call.
	 * @param mpcController: The optimized control policy of MPC.
	 * @param mpcTimeTrajectory: The optimized time trajectory of the policy message on the buffer.
	 * @param mpcStateTrajectory: The optimized state trajectory of the policy message on the buffer.
	 * @param eventTimes: The event times of the policy.
	 * @param subsystemsSequence: The subsystems sequence of the policy.
	 */
	virtual void modifyPolicy(
			bool& logicUpdated,
			controller_t& mpcController,
			scalar_array_t& mpcTimeTrajectory,
			state_vector_array_t& mpcStateTrajectory,
			scalar_array_t& eventTimes,
			size_array_t& subsystemsSequence) override;

	/**
	 * Updates feet trajectories.
	 *
	 * @param [in] eventTimes: Event times.
	 * @param [in] subsystemsSequence: Subsystems sequence.
	 * @param [in] touchdownTimeStock: An array of touch-down times.
	 * @param [in] touchdownStateStock: An array of feet positions at touch-down.
	 * @param [in] touchdownInputStock: An array of feet velocities at touch-down.
	 */
	virtual void updateFeetTrajectories(
			const scalar_array_t& eventTimes,
			const size_array_t& subsystemsSequence,
			const scalar_array_t& touchdownTimeStock,
			const state_vector_array_t& touchdownStateStock,
			const input_vector_array_t& touchdownInputStock);

	/**
	 * Computes swing phase progress.
	 *
	 * @param [in] activeSubsystemIndex: Active subsystem index
	 * @param [in] stanceLegs: Stance leg flags
	 * @param [in] time: current time.
	 * @param [out] swingPhaseProgress: swing phase progress for legs.
	 */
	void computeSwingPhaseProgress(
			const size_t& activeSubsystemIndex,
			const contact_flag_t& stanceLegs,
			const scalar_t& time,
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
	virtual void computeFeetState(
			const state_vector_t& state,
			const input_vector_t& input,
			vector_3d_array_t& o_feetPosition,
			vector_3d_array_t& o_feetVelocity,
			vector_3d_array_t& o_contactForces);

private:
	/*
	 * Variables
	 */
	quadruped_interface_ptr_t ocs2QuadrupedInterfacePtr_;
  	logic_rules_ptr_t logic_rules_mrt_;

	Model_Settings modelSettings_;

	std::array<const cpg_t*,4> feetZPlanPtr_;
	std::array<scalar_t, 4> swingPhaseProgress_;

	std::vector<std::array<cubic_spline_ptr_t,4>> feetXPlanPtrStock_;
	std::vector<std::array<cubic_spline_ptr_t,4>> feetYPlanPtrStock_;

	scalar_array_t			touchdownTimeStockBuffer_;
	state_vector_array_t 	touchdownStateStockBuffer_;
	input_vector_array_t 	touchdownInputStockBuffer_;
};

} // end of namespace switched_model

#include "implementation/MRT_ROS_Quadruped.h"


#endif /* MRT_ROS_QUADRUPED_H_ */
