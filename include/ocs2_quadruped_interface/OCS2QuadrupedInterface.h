/*
 * OCS2QuadrupedInterface.h
 *
 *  Created on: Feb 14, 2018
 *      Author: farbod
 */

#ifndef OCS2QUADRUPEDINTERFACE_H_
#define OCS2QUADRUPEDINTERFACE_H_

#include <array>
#include <memory>
#include <csignal>
#include <chrono>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include<Eigen/StdVector>
#include <vector>

#include <ocs2_core/misc/loadEigenMatrix.h>

#include <ocs2_slq/SLQ.h>
#include <ocs2_slq/SLQ_MP.h>
//#include <ocs2_ocs2/OCS2Projected.h>

#include <ocs2_mpc/MPC_SLQ.h>
#include <ocs2_mpc/MPC_OCS2.h>

#include <ocs2_robotic_examples/common/RobotInterfaceBase.h>

#include <ocs2_switched_model_interface/core/Model_Settings.h>
#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/ground/FlatGroundProfile.h>
#include <ocs2_switched_model_interface/logic/SwitchedModelLogicRulesBase.h>
#include <ocs2_switched_model_interface/misc/WeightCompensationForces.h>
#include <ocs2_switched_model_interface/state_constraint/EndEffectorConstraintBase.h>
#include <ocs2_switched_model_interface/constraint/ComKinoConstraintBase.h>
#include <ocs2_switched_model_interface/core/SwitchedModelStateEstimator.h>
#include <ocs2_switched_model_interface/cost/SwitchedModelCostBase.h>
#include <ocs2_switched_model_interface/dynamics/ComKinoDynamicsBase.h>
#include <ocs2_switched_model_interface/dynamics_derivative/ComKinoDynamicsDerivativeBase.h>
#include <ocs2_switched_model_interface/foot_planner/cpg/SplineCPG.h>
#include <ocs2_switched_model_interface/foot_planner/FeetZDirectionPlanner.h>
#include <ocs2_switched_model_interface/initialization/ComKinoOperatingPointsBase.h>
#include <ocs2_switched_model_interface/state_constraint/EllipticalConstraint.h>
#include <ocs2_switched_model_interface/state_constraint/EndEffectorConstraintsUtilities.h>


namespace switched_model {

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM = 12+JOINT_COORD_SIZE, size_t INPUT_DIM = 12+JOINT_COORD_SIZE>
class OCS2QuadrupedInterface : public ocs2::RobotInterfaceBase<STATE_DIM, INPUT_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum
	{
		state_dim_ = STATE_DIM,
		input_dim_ = INPUT_DIM,
		rbd_state_dim_ = 12 + 2*JOINT_COORD_SIZE
	};

	typedef ocs2::RobotInterfaceBase<STATE_DIM, INPUT_DIM> BASE;

	typedef std::shared_ptr<OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>> Ptr;

	typedef ComModelBase<JOINT_COORD_SIZE> com_model_t;
	typedef KinematicsModelBase<JOINT_COORD_SIZE> kinematic_model_t;

	typedef SwitchedModelStateEstimator<JOINT_COORD_SIZE>	state_estimator_t;

	typedef typename BASE::DIMENSIONS dimension_t;
	typedef typename dimension_t::scalar_t		 			scalar_t;
	typedef typename dimension_t::scalar_array_t 			scalar_array_t;
	typedef typename dimension_t::size_array_t	 			size_array_t;
	typedef typename dimension_t::controller_t		 		controller_t;
	typedef typename dimension_t::controller_array_t 		controller_array_t;
	typedef typename dimension_t::state_vector_t 			state_vector_t;
	typedef typename dimension_t::state_vector_array_t 		state_vector_array_t;
	typedef typename dimension_t::state_vector_array2_t 	state_vector_array2_t;
	typedef typename dimension_t::input_vector_t 		  	input_vector_t;
	typedef typename dimension_t::input_vector_array_t  	input_vector_array_t;
	typedef typename dimension_t::input_vector_array2_t 	input_vector_array2_t;
	typedef typename dimension_t::eigen_scalar_t			eigen_scalar_t;
	typedef typename dimension_t::eigen_scalar_array_t		eigen_scalar_array_t;
	typedef typename dimension_t::state_matrix_t			state_matrix_t;
	typedef typename dimension_t::input_matrix_t			input_matrix_t;
	typedef typename dimension_t::input_state_matrix_t 	   	input_state_matrix_t;
	typedef typename dimension_t::input_state_matrix_array_t 	input_state_matrix_array_t;

	typedef ocs2::CostDesiredTrajectories<scalar_t> 		cost_desired_trajectories_t;

	typedef SwitchedModel<JOINT_COORD_SIZE> 					switched_model_t;
	typedef typename switched_model_t::contact_flag_t 			contact_flag_t;
	typedef typename switched_model_t::generalized_coordinate_t generalized_coordinate_t;
	typedef typename switched_model_t::joint_coordinate_t 		joint_coordinate_t;
	typedef typename switched_model_t::base_coordinate_t 		base_coordinate_t;
	typedef Eigen::Matrix<scalar_t, rbd_state_dim_,1>			rbd_state_vector_t;

	typedef GroundProfileBase<scalar_t> 		ground_profile_t;
	typedef typename ground_profile_t::Ptr 		ground_profile_ptr_t;
	typedef FlatGroundProfile<scalar_t> 		flat_ground_profile_t;
	typedef typename flat_ground_profile_t::Ptr flat_ground_profile_ptr_t;

	typedef	SplineCPG<scalar_t>						cpg_t;
	typedef FeetZDirectionPlanner<scalar_t, cpg_t>	feet_z_planner_t;
	typedef typename feet_z_planner_t::Ptr			feet_z_planner_ptr_t;

	typedef SwitchedModelPlannerLogicRules<JOINT_COORD_SIZE> 	logic_rules_t;
	typedef typename logic_rules_t::Ptr logic_rules_ptr_t;

	typedef ocs2::ModeSequenceTemplate<scalar_t> mode_sequence_template_t;

	typedef ocs2::SLQ_BASE<STATE_DIM, INPUT_DIM, logic_rules_t>	slq_base_t;
	typedef ocs2::SLQ<STATE_DIM, INPUT_DIM, logic_rules_t>  	slq_t;
	typedef ocs2::SLQ_MP<STATE_DIM, INPUT_DIM, logic_rules_t>  	slq_mp_t;
//	typedef ocs2::OCS2Projected<STATE_DIM, INPUT_DIM> 			ocs2_t;
	typedef ocs2::MPC_SLQ<STATE_DIM, INPUT_DIM, logic_rules_t>	mpc_t;

	typedef typename slq_base_t::Ptr 	slq_base_ptr_t;
	typedef typename slq_t::Ptr  		slq_ptr_t;
	typedef typename slq_mp_t::Ptr  	slq_mp_ptr_t;
//	typedef typename ocs2_t::Ptr 		ocs2_ptr_t;
	typedef typename mpc_t::Ptr 		mpc_ptr_t;

	/**
	 * Default constructor
	 */
	OCS2QuadrupedInterface() = delete;

	/**
	 * Constructor
	 *
	 * @param pathToConfigFolder: Path to configuration folder
	 */
	OCS2QuadrupedInterface(
			const kinematic_model_t& kinematicModel,
			const com_model_t& comModel,
			const std::string& pathToConfigFolder);

	/**
	 * Destructor
	 */
	virtual ~OCS2QuadrupedInterface() {}

	/**
	 * setup all optimizes
	 */
	virtual void setupOptimizer(
			const logic_rules_ptr_t& logicRulesPtr,
			const mode_sequence_template_t* modeSequenceTemplatePtr,
			slq_base_ptr_t& slqPtr,
			mpc_ptr_t& mpcPtr) = 0;

	/**
	 * Designs weight compensating input.
	 *
	 * @param [in] switchedState: Switched model state.
	 * @param [out] uForWeightCompensation: Weight compensating input.
	 */
	virtual void designWeightCompensatingInput(
			const state_vector_t& switchedState,
			input_vector_t& uForWeightCompensation) = 0;

	/**
	 * Run the SLQ algorithm.
	 *
	 * @param initTime: Initial time.
	 * @param initState: Initial robot's RBD state.
	 * @param finalTime: Final time.
	 * @param initialControllersStock: Initial controller (optional).
	 */
	void runSLQ (
			const scalar_t& initTime,
			const rbd_state_vector_t& initState,
			const scalar_t& finalTime,
			const controller_array_t& initialControllersStock = controller_array_t());

	/**
	 * Run the SLQ-MPC algorithm.
	 *
	 * @param initTime: Initial time.
	 * @param initState: Initial robot's RBD state.
	 * @return
	 */
	bool runMPC(
			const scalar_t& initTime,
			const rbd_state_vector_t& initState);

	/**
	 * Run OCS2.
	 *
	 * @param initTime
	 * @param initState
	 * @param switchingTimes
	 */
	void runOCS2(
			const scalar_t& initTime,
			const rbd_state_vector_t& initState,
			const scalar_array_t& switchingTimes = scalar_array_t());

	/**
	 * Gets a const reference to the internal logicRules.
	 *
	 * @return const reference to the internal logicRules.
	 */
	logic_rules_t& getLogicRules();

	/**
	 * Updates the logicMachine.
	 *
	 * @param [in] partitioningTimes: partitioning times array.
	 * @param [in] logicRulesUpdated: If the internal logicRules class has been already updated.
	 */
	void updateLogicMachine(
			const scalar_array_t& partitioningTimes,
			bool logicRulesUpdated = true);

	/**
	 * Computes switched model state from the RBD state
	 *
	 * @param [in] rbdState: RBD state
	 * @param [out] comkinoState: Switched model state.
	 */
	virtual void computeSwitchedModelState(
			const rbd_state_vector_t& rbdState,
			state_vector_t& comkinoState);

	/**
	 * Computes the RBD state from the switched model state.
	 *
	 * @param [in] comkinoState: switched model state.
	 * @param [in] comkinoInput: switched model input.
	 * @param [out] rbdState: RBD state
	 */
	void computeRbdModelState(
			const state_vector_t& comkinoState,
			const input_vector_t& comkinoInput,
			rbd_state_vector_t& rbdState);

	/**
	 * Computes the CoM's local acceleration about the CoM frame.
	 * Note that here the off-diagonal blocks of the inertia tensor is zero.
	 *
	 * @param [in] comkinoState: CoM-Kino model state.
	 * @param [in] comkinoInput: CoM-Kino model input.
	 * @param [out] comLocalAcceleration: CoM acceleration about the CoM frame.
	 */
	void computeComLocalAcceleration(
			const state_vector_t& comkinoState,
			const input_vector_t& comkinoInput,
			base_coordinate_t& comLocalAcceleration);

	/**
	 * Computes the CoM's pose, velocity, and acceleration in the origin frame from the switched model state and input.
	 *
	 * @param [in] comkinoState: CoM-Kino model state.
	 * @param [in] comkinoInput: CoM-Kino model input.
	 * @param [out] o_comPose: CoM pose in the origin frame.
	 * @param [out] o_comVelocity: CoM velocity in the origin frame.
	 * @param [out] o_comAcceleration: CoM acceleration in the origin frame.
	 */
	void computeComStateInOrigin(
			const state_vector_t& comkinoState,
			const input_vector_t& comkinoInput,
			base_coordinate_t& o_comPose,
			base_coordinate_t& o_comVelocity,
			base_coordinate_t& o_comAcceleration);

	/**
	 * Estimates flat ground hight.
	 *
	 * @param [in] rbdState: RBD state.
	 * @param [in] contactFlag: Contact flag.
	 * @param [out] groundHight: Ground hight
	 */
	void estimateFlatGround(
			const rbd_state_vector_t& rbdState,
			const contact_flag_t& contactFlag,
			scalar_t& groundHight) const;

	/**
	 * Get a reference to the robot kinematic model.
	 *
	 * @return A reference to the robot kinematic model
	 */
	kinematic_model_t& getKinematicModel();

	/**
	 * Get a reference to the robot CoM model.
	 *
	 * @return A reference to the robot CoM model
	 */
	com_model_t& getComModel();

	/**
	 * Gets a reference to the internal SLQ class.
	 *
	 * @return Reference to the internal SLQ
	 */
	slq_base_t& getSLQ();

	/**
	 * Gets a pointer to the internal SLQ class.
	 *
	 * @return Pointer to the internal SLQ
	 */
	slq_base_ptr_t& getSLQPtr();

	/**
	 * Gets a reference to the internal SLQ-MPC class.
	 *
	 * @return Reference to the internal MPC
	 */
	mpc_t& getMPC();

	/**
	 * Gets a pointer to the internal SLQ-MPC class.
	 *
	 * @return Pointer to the internal MPC
	 */
	mpc_ptr_t& getMPCPtr();

	/**
	 * Gets the cost function and ISEs of the type-1 and type-2 constraints at the initial time.
	 *
	 * @param [out] costFunction: cost function value
	 * @param [out] constraint1ISE: type-1 constraint ISE.
	 * @param [out] constraint1ISE: type-2 constraint ISE.
	 */
	void getPerformanceIndeces(
			scalar_t& costFunction,
			scalar_t& constriantISE1,
			scalar_t& constriantISE2) const;

	/**
	 * Gets Iterations Log of SLQ.
	 *
	 * @param [out] iterationCost: Each iteration's cost.
	 * @param [out] iterationISE1: Each iteration's type-1 constraints ISE.
	 * @param [out] iterationISE2: Each iteration's type-2 constraints ISE.
	 */
	void getIterationsLog(
			eigen_scalar_array_t& iterationCost,
			eigen_scalar_array_t& iterationISE1,
			eigen_scalar_array_t& ocs2Iterationcost) const;

	/**
	 * Gets a pointer to the optimized array of the control policies.
	 *
	 * @param [out] controllersStockPtr: A pointer to the optimal array of the control policies
	 */
	void getOptimizedControllerPtr(const controller_array_t*& controllersStockPtr) const;

	/**
	 * Gets a pointer to the optimized time, state, and input trajectories.
	 *
	 * @param [out] timeTrajectoriesStockPtr: A pointer to an array of trajectories containing the output time trajectory stamp.
	 * @param [out] stateTrajectoriesStockPtr: A pointer to an array of trajectories containing the output state trajectory.
	 * @param [out] inputTrajectoriesStockPtr: A pointer to an array of trajectories containing the output control input trajectory.
	 */
	void getOptimizedTrajectoriesPtr(
			const std::vector<scalar_array_t>*& timeTrajectoriesStockPtr,
			const state_vector_array2_t*& stateTrajectoriesStockPtr,
			const input_vector_array2_t*& inputTrajectoriesStockPtr) const ;

	/**
	 * Get events times.
	 *
	 * @param [in] eventTimesPtr: a pointer to eventTimesPtr_
	 */
	void getEventTimesPtr(const scalar_array_t*& eventTimesPtr) const;

	/**
	 * Get motion subsystems sequence.
	 *
	 * @param [in] motionPhasesSequencePtr: a pointer to motionPhasesSequencePtr_
	 */
	void getSubsystemsSequencePtr(const size_array_t*& motionPhasesSequencePtr) const;

	/**
	 * Get contact flags sequence
	 *
	 * @param [in] contactFlagsSequencePtr: a pointer to contactFlagsSequencePtr_
	 */
	void getContactFlagsSequencePtr(const std::vector<contact_flag_t>*& contactFlagsSequencePtr) const;

	/**
	 * Get an array of pointers to the loaded gap class.
	 *
	 * @param gapIndicatorPtrs
	 */
	void getGapIndicatorPtrs(std::vector<EndEffectorConstraintBase::ConstPtr>& gapIndicatorPtrs) const;

	/**
	 * Get SLQ settings.
	 *
	 * @return SLQ settings.
	 */
	Model_Settings& modelSettings();

	/**
	 * Gets the loaded initial RBD state.
	 *
	 * @param initRbdState: initial RBD state.
	 */
	void getLoadedInitialState(rbd_state_vector_t& initRbdState) const;

	/**
	 * Gets the time horizon of the MPC.
	 *
	 * @param [out] timeHorizon: The time horizon of the MPC.
	 */
	void getLoadedTimeHorizon(scalar_t& timeHorizon) const;

	/**
	 * This function loads the simulation-specific settings: dt, tFinal, initSettlingTime
	 *
	 * @param filename
	 * @param dt
	 * @param tFinal
	 * @param initSettlingTime
	 */
	static void loadSimulationSettings(const std::string& filename,
			scalar_t& dt,
			scalar_t& tFinal,
			scalar_t& initSettlingTime);

	/**
	 * This function loads the visualization-specific settings: slowdown factor, visualization time
	 *
	 * @param filename
	 * @param slowdown
	 * @param vizTime
	 */
	static void loadVisualizationSettings(const std::string& filename, scalar_t& slowdown, scalar_t& vizTime);


	double strideTime() {
		return modelSettings_.strideTime_;
	}

	double strideLength() {
		return modelSettings_.mpcStrideLength_;
	}

	size_t numPhasesInfullGaitCycle() {
		return modelSettings_.numPhasesInfullGaitCycle_;
	}

protected:
	/**
	 * Load the settings from the path file.
	 *
	 * @param pathToConfigFile
	 */
	void loadSettings(const std::string& pathToConfigFile) final;

	/**
	 * Setups all optimizers which you require.
	 *
	 * @param [in] taskFile: Task's file full path.
	 */
	void setupOptimizer(const std::string& taskFile) final {

		throw std::runtime_error("It has been overloaded.");
	}

	/**
	 * concatenate the container stocks
	 */
	void concatenate();

/*
 * Variables
 */

	typename kinematic_model_t::Ptr kinematicModelPtr_;
	typename com_model_t::Ptr comModelPtr_;

	state_estimator_t switchedModelStateEstimator_;

	logic_rules_ptr_t logicRulesPtr_; // logic

	Model_Settings modelSettings_;

	ground_profile_ptr_t groundProfilePtr_;

	state_matrix_t Q_;
	input_matrix_t R_;
	state_matrix_t QFinal_;
	state_vector_t xFinal_;

	scalar_t impulseWeight_;
	scalar_t impulseSigmeFactor_;

	std::vector<EndEffectorConstraintBase::ConstPtr> gapIndicatorPtrs_;

	rbd_state_vector_t 	initRbdState_;
	using BASE::initialState_;

	scalar_t        initTime_;
	scalar_t        finalTime_;
	scalar_t        timeHorizon_;
	size_t          numPartitions_;
	scalar_array_t  partitioningTimes_;

	size_t 		   initNumSubsystems_;
	scalar_array_t initEventTimes_;
	size_array_t   initSwitchingModes_;
	std::vector<contact_flag_t> initStanceLegSequene_;

	size_t 		   numSubsystems_;
	scalar_array_t switchingTimes_;
	size_array_t   switchingModes_;
	std::vector<contact_flag_t> stanceLegSequene_;

	mode_sequence_template_t modeSequenceTemplate_;

	scalar_t costFunction_;
	scalar_t constriantISE1_;
	scalar_t constriantISE2_;

	eigen_scalar_array_t iterationCost_;
	eigen_scalar_array_t iterationISE1_;
	eigen_scalar_array_t iterationISE2_;
	eigen_scalar_array_t ocs2Iterationcost_;

	// SLQ
	slq_base_ptr_t	slqPtr_;
	// OCS2
//	ocs2_ptr_t 		ocs2Ptr_;
	// MPC
	mpc_ptr_t 		mpcPtr_;

	scalar_array_t 				eventTimes_;
	size_array_t				subsystemsSequence_;
	std::vector<contact_flag_t> contactFlagsSequence_;

	const controller_array_t* 			controllersStockPtr_;
	const std::vector<scalar_array_t>* 	timeTrajectoriesStockPtr_;
	const state_vector_array2_t* 		stateTrajectoriesStockPtr_;
	const input_vector_array2_t* 		inputTrajectoriesStockPtr_;

	cost_desired_trajectories_t	costDesiredTrajectories_;

	scalar_array_t 			timeTrajectory_;
	state_vector_array_t 	stateTrajectory_;
	input_vector_array_t 	inputTrajectory_;
	scalar_array_t 			   controllerTimeTrajectory_;
	input_vector_array_t 	   controllerFFTrajector_;
	input_state_matrix_array_t controllerFBTrajectory_;

	ocs2::LinearInterpolation<state_vector_t, Eigen::aligned_allocator<state_vector_t> > 		linInterpolateState_;
	ocs2::LinearInterpolation<input_vector_t, Eigen::aligned_allocator<input_vector_t> > 		linInterpolateInput_;
	ocs2::LinearInterpolation<input_vector_t, Eigen::aligned_allocator<input_vector_t> > 		linInterpolateUff_;
	ocs2::LinearInterpolation<input_state_matrix_t, Eigen::aligned_allocator<input_state_matrix_t>> linInterpolateK_;

};

} // end of namespace switched_model

#include "implementation/OCS2QuadrupedInterface.h"

#endif /* OCS2QUADRUPEDINTERFACE_H_ */
