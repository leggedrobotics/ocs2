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

#include <ocs2_slq/GLQP.h>
#include <ocs2_slq/SLQ.h>
#include <ocs2_slq/SLQ_MP.h>
//#include <ocs2_ocs2/OCS2Projected.h>

#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_mpc/MPC_SLQ.h>

#include <c_switched_model_interface/core/Model_Settings.h>
#include <c_switched_model_interface/core/SwitchedModel.h>
#include <c_switched_model_interface/core/MotionPhaseDefinition.h>
#include <c_switched_model_interface/core/SwitchedModelStateEstimator.h>

#include <c_switched_model_interface/logic/SwitchedModelLogicRulesBase.h>
#include <c_switched_model_interface/dynamics/ComKinoDynamicsBase.h>
#include <c_switched_model_interface/dynamics_derivative/ComKinoDynamicsDerivativeBase.h>
#include <c_switched_model_interface/constraint/ComKinoConstraintBase.h>
#include <c_switched_model_interface/cost/SwitchedModelCostBase.h>
#include <c_switched_model_interface/initialization/ComKinoOperatingPointsBase.h>

#include <c_switched_model_interface/misc/WeightCompensationForces.h>

#include <c_switched_model_interface/state_constraint/EndEffectorConstraintBase.h>
#include <c_switched_model_interface/state_constraint/EllipticalConstraint.h>
#include <c_switched_model_interface/state_constraint/EndEffectorConstraintsUtilities.h>


namespace switched_model {

template <size_t JOINT_COORD_SIZE>
class OCS2QuadrupedInterface
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum
	{
		STATE_DIM = 12+JOINT_COORD_SIZE,
		INPUT_DIM = 12+JOINT_COORD_SIZE,
		RBD_STATE_DIM = 12 + 2*JOINT_COORD_SIZE
	};

	typedef std::shared_ptr<OCS2QuadrupedInterface> Ptr;

	typedef ocs2::Dimensions<STATE_DIM,INPUT_DIM> dimension_t;

	typedef ComModelBase<JOINT_COORD_SIZE> com_model_t;
	typedef KinematicsModelBase<JOINT_COORD_SIZE> kinematic_model_t;

	typedef SwitchedModelLogicRulesBase<JOINT_COORD_SIZE> 	logic_rules_t;
	typedef SwitchedModelStateEstimator<JOINT_COORD_SIZE>	state_estimator_t;

	typedef ocs2::GLQP<STATE_DIM, INPUT_DIM, logic_rules_t>		lq_t;
	typedef ocs2::SLQ_BASE<STATE_DIM, INPUT_DIM, logic_rules_t>	slqp_base_t;
	typedef ocs2::SLQ<STATE_DIM, INPUT_DIM, logic_rules_t>  	slqp_t;
	typedef ocs2::SLQ_MP<STATE_DIM, INPUT_DIM, logic_rules_t>  	slqp_mp_t;
//	typedef ocs2::OCS2Projected<STATE_DIM, INPUT_DIM> 			ocs2_t;
	typedef ocs2::MPC_SLQ<STATE_DIM, INPUT_DIM, logic_rules_t>	mpc_t;

	typedef typename lq_t::Ptr 			lq_ptr_t;
	typedef typename slqp_base_t::Ptr 	slqp_base_ptr_t;
	typedef typename slqp_t::Ptr  		slqp_ptr_t;
	typedef typename slqp_mp_t::Ptr  	slqp_mp_ptr_t;
//	typedef typename ocs2_t::Ptr 		ocs2_ptr_t;
	typedef typename mpc_t::Ptr 		mpc_ptr_t;

	typedef typename dimension_t::scalar_t		 scalar_t;
	typedef typename dimension_t::scalar_array_t scalar_array_t;
	typedef typename dimension_t::size_array_t	 size_array_t;
	typedef typename dimension_t::controller_t		 controller_t;
	typedef typename dimension_t::controller_array_t controller_array_t;
	typedef typename dimension_t::state_vector_t 		state_vector_t;
	typedef typename dimension_t::state_vector_array_t 	state_vector_array_t;
	typedef typename dimension_t::state_vector_array2_t state_vector_array2_t;
	typedef typename dimension_t::control_vector_t 		  input_vector_t;
	typedef typename dimension_t::control_vector_array_t  input_vector_array_t;
	typedef typename dimension_t::control_vector_array2_t input_vector_array2_t;
	typedef typename dimension_t::eigen_scalar_t		eigen_scalar_t;
	typedef typename dimension_t::eigen_scalar_array_t	eigen_scalar_array_t;
	typedef typename dimension_t::state_matrix_t	state_matrix_t;
	typedef typename dimension_t::control_matrix_t	control_matrix_t;
	typedef typename dimension_t::control_feedback_t 	   control_feedback_t;
	typedef typename dimension_t::control_feedback_array_t control_feedback_array_t;

	typedef SwitchedModel<JOINT_COORD_SIZE> switched_model_t;
	typedef typename switched_model_t::contact_flag_t 			contact_flag_t;
	typedef typename switched_model_t::generalized_coordinate_t generalized_coordinate_t;
	typedef typename switched_model_t::joint_coordinate_t 		joint_coordinate_t;
	typedef typename switched_model_t::base_coordinate_t 		base_coordinate_t;
	typedef Eigen::Matrix<scalar_t, RBD_STATE_DIM,1>			rbd_state_vector_t;

	/**
	 * Default constructor
	 */
	OCS2QuadrupedInterface() = delete;

	/**
	 * Constructor
	 *
	 * @param pathToConfigFolder: Path to config folder
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
	virtual void setupOptimizer(const logic_rules_t& logicRules,
			slqp_base_ptr_t& slqPtr) = 0;

	/**
	 * Designs desired trajectories for each time partition.
	 *
	 * @param [out] desiredTimeTrajectoriesStock: The desired time trajectories.
	 * @param [out] desiredStateTrajectoriesStock: The desired state trajectories.
	 * @param [out] desiredInputTrajectoriesStock: The desired input trajectories.
	 */
	virtual void designDesiredTrajectories(
				std::vector<scalar_array_t>& desiredTimeTrajectoriesStock,
				state_vector_array2_t& desiredStateTrajectoriesStock,
				input_vector_array2_t& desiredInputTrajectoriesStock) = 0;

	/**
	 * Run the SLQ algorithm.
	 *
	 * @param initTime: Initial time.
	 * @param initState: Initial robot's RBD state.
	 * @param finalTime: Final time.
	 * @param partitioningTimes: Time partitioning.
	 * @param initialControllersStock: Initial controller (optional).
	 */
	void runSLQ (
			const scalar_t& initTime,
			const rbd_state_vector_t& initState,
			const scalar_t& finalTime,
			const scalar_array_t& partitioningTimes = scalar_array_t(),
			const controller_array_t& initialControllersStock = controller_array_t());

	/**
	 * Run the SLQ-MPC algorithm.
	 *
	 * @param initTime: Initial time.
	 * @param initState: Initial robot's RBD state.
	 * @return
	 */
	bool runMPC(const scalar_t& initTime,
			const rbd_state_vector_t& initState);

	/**
	 * Run OCS2.
	 *
	 * @param initTime
	 * @param initState
	 * @param switchingTimes
	 */
	void runOCS2(const scalar_t& initTime,
			const rbd_state_vector_t& initState,
			const scalar_array_t& switchingTimes = scalar_array_t());

	/**
	 * Computes switched model state from the RBD state
	 *
	 * @param rbdState: RBD state
	 * @param comkinoState: Switched model state.
	 */
	void computeSwitchedModelState(const rbd_state_vector_t& rbdState,
			state_vector_t& comkinoState);

	/**
	 * Set new goal to MPC.
	 *
	 * @param newGoalDuration
	 * @param newGoalState
	 */
	void setNewGoalStateMPC(const scalar_t& newGoalDuration,
			const state_vector_t& newGoalState);

	/**
	 * Get performance indeces.
	 *
	 * @param costFunction
	 * @param constriantISE1
	 * @param constriantISE2
	 */
	void getPerformanceIndeces(scalar_t& costFunction,
			scalar_t& constriantISE1,
			scalar_t& constriantISE2) const;

	/**
	 * Get the log iterations
	 *
	 * @param iterationCost
	 * @param iterationISE1
	 * @param ocs2Iterationcost
	 */
	void getIterationsLog(eigen_scalar_array_t& iterationCost,
			eigen_scalar_array_t& iterationISE1,
			eigen_scalar_array_t& ocs2Iterationcost) const;
	/**
	 * Get switching times.
	 *
	 * @param switchingTimes
	 */
	void getSwitchingTimes(scalar_array_t& switchingTimes) const;

	/**
	 * Get controller
	 *
	 * @param controllersStockPtr
	 */
	void getController(controller_array_t& controllersStockPtr) const;

	/**
	 * Get controller pointer.
	 *
	 * @return
	 */
	std::shared_ptr<const controller_array_t> getControllerPtr() const;

	/**
	 * Get optimized trajectories.
	 *
	 * @param timeTrajectoriesStock
	 * @param stateTrajectoriesStock
	 * @param inputTrajectoriesStock
	 */
	void getTrajectories(std::vector<scalar_array_t>& timeTrajectoriesStock,
			state_vector_array2_t& stateTrajectoriesStock,
			input_vector_array2_t& inputTrajectoriesStock) const;

	/**
	 * Get a pointer to the optimized trajectories time stamp.
	 *
	 * @return
	 */
	std::shared_ptr<const std::vector<scalar_array_t>> getTimeTrajectoriesPtr() const;

	/**
	 * Get a pointer to the optimized state trajectory.
	 *
	 * @return
	 */
	std::shared_ptr<const state_vector_array2_t> getStateTrajectoriesPtr() const;

	/**
	 * Get a pointer to the optimized input trajectory.
	 *
	 * @return
	 */
	std::shared_ptr<const input_vector_array2_t> getInputTrajectoriesPtr() const;

	/**
	 * Get stance legs sequene.
	 *
	 * @param stanceLegSequene
	 */
	void getStanceLegSequene(std::vector<contact_flag_t>& stanceLegSequene) const;

	/**
	 * Get gait sequene.
	 *
	 * @param gait sequence
	 */
	void getGaitSequence(size_array_t& gaitSequence) const;

	/**
	 * Get an array of pointers to the loaded gap class.
	 *
	 * @param gapIndicatorPtrs
	 */
	void getGapIndicatorPtrs(std::vector<EndEffectorConstraintBase::ConstPtr>& gapIndicatorPtrs) const;

	/**
	 * Get MPC settings.
	 *
	 * @return MPC settings.
	 */
	ocs2::MPC_Settings& getMpcSettings();


	/**
	 * Get SLQ settings.
	 *
	 * @return SLQ settings.
	 */
	ocs2::SLQ_Settings& getSlqSettings();

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
	 * This function loads the visualization-specific settings: slowdown factor, vizualization time
	 *
	 * @param filename
	 * @param slowdown
	 * @param vizTime
	 */
	static void loadVisualizationSettings(const std::string& filename, scalar_t& slowdown, scalar_t& vizTime);


	double strideTime() {
		return (initSwitchingTimes_[1]-initSwitchingTimes_[0]);
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
	void loadSettings(const std::string& pathToConfigFile);

	/**
	 * Get optimizers parameters
	 *
	 * @param optimizerPtr
	 */
	template<class T>
	void getOptimizerParameters(const std::shared_ptr<T>& optimizerPtr);

	/**
	 * concatenate the contatiner stocks
	 */
	void concatenate();

/*
 * Variables
 */

	typename kinematic_model_t::Ptr kinematicModelPtr_;
	typename com_model_t::Ptr comModelPtr_;

	state_estimator_t switchedModelStateEstimator_;

	logic_rules_t logicRules_; // logic

	ocs2::SLQ_Settings slqSettings_;
	ocs2::MPC_Settings mpcSettings_;

	Model_Settings modelSettings_;

	state_matrix_t Q_;
	control_matrix_t R_;
	state_matrix_t 	QFinal_;
	state_vector_t 	xFinal_;

	scalar_t impulseWeight_;
	scalar_t impulseSigmeFactor_;

	std::vector<EndEffectorConstraintBase::ConstPtr> gapIndicatorPtrs_;

	rbd_state_vector_t 	initRbdState_;
	state_vector_t initSwitchedState_;

	scalar_t 		initTime_;
	scalar_t 		finalTime_;
	scalar_array_t	partitioningTimes_;
	size_t 		    numPartitioningTimes_;

	size_t 		   initNumSubsystems_;
	scalar_array_t initSwitchingTimes_;
	size_array_t   initSwitchingModes_;
	std::vector<contact_flag_t> initStanceLegSequene_;

	size_t 		   numSubsystems_;
	scalar_array_t switchingTimes_;
	size_array_t   switchingModes_;
	std::vector<contact_flag_t> stanceLegSequene_;

	scalar_t costFunction_;
	scalar_t constriantISE1_;
	scalar_t constriantISE2_;

	eigen_scalar_array_t iterationCost_;
	eigen_scalar_array_t iterationISE1_;
	eigen_scalar_array_t iterationISE2_;
	eigen_scalar_array_t ocs2Iterationcost_;

	//
	lq_ptr_t lqPtr_;
	// SLQ
	slqp_base_ptr_t	slqPtr_;
	// OCS2
//	ocs2_ptr_t 		ocs2Ptr_;
	// MPC
	mpc_ptr_t 		mpcPtr_;

	controller_array_t 			controllersStock_;
	std::vector<scalar_array_t> timeTrajectoriesStock_;
	state_vector_array2_t 		stateTrajectoriesStock_;
	input_vector_array2_t 		inputTrajectoriesStock_;

	std::shared_ptr<const controller_array_t> 			controllersStockPtr_;
	std::shared_ptr<const std::vector<scalar_array_t>> 	timeTrajectoriesStockPtr_;
	std::shared_ptr<const state_vector_array2_t> 		stateTrajectoriesStockPtr_;
	std::shared_ptr<const input_vector_array2_t> 		inputTrajectoriesStockPtr_;

	std::vector<scalar_array_t>	desiredTimeTrajectoriesStock_;
	state_vector_array2_t 		desiredStateTrajectoriesStock_;
	input_vector_array2_t 		desiredInputTrajectoriesStock_;

	scalar_array_t 			timeTrajectory_;
	state_vector_array_t 	stateTrajectory_;
	input_vector_array_t 	inputTrajectory_;
	scalar_array_t 			controllerTimeTrajectory_;
	control_feedback_array_t controllerFBTrajectory_;
	input_vector_array_t 	controllerFFTrajector_;

	ocs2::LinearInterpolation<state_vector_t, Eigen::aligned_allocator<state_vector_t> > 		linInterpolateState_;
	ocs2::LinearInterpolation<input_vector_t, Eigen::aligned_allocator<input_vector_t> > 		linInterpolateInput_;
	ocs2::LinearInterpolation<input_vector_t, Eigen::aligned_allocator<input_vector_t> > 		linInterpolateUff_;
	ocs2::LinearInterpolation<control_feedback_t, Eigen::aligned_allocator<control_feedback_t>> linInterpolateK_;

};

} // end of namespace switched_model

#include "implementation/OCS2QuadrupedInterface.h"

#endif /* OCS2QUADRUPEDINTERFACE_H_ */
