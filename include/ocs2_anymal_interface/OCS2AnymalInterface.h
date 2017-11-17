/*
 * OCS2AnymalInterface.h
 *
 *  Created on: Sep 4, 2016
 *      Author: farbod
 */

#ifndef OCS2ANYMALINTERFACE_H_
#define OCS2ANYMALINTERFACE_H_

#include <array>
#include <memory>
#include <vector>
#include <csignal>
#include <chrono>
#include <iostream>
#include <string>
#include <Eigen/Dense>

#include <ocs2_slq/LQP.h>
#include <ocs2_slq/GLQP.h>
#include <ocs2_slq/SLQP.h>
#include <ocs2_slq/SLQP_MP.h>
#include <ocs2_ocs2/OCS2Projected.h>
#include <ocs2_core/misc/LinearInterpolation.h>

// #include <mpc/MPC_SLQP.h>
// #include <mpc/util/LoadSettings_MPC.h>

#include <ocs2_anymal_interface/LoadTask.h>
#include <c_switched_model_interface/misc/SinCpg.h>
#include <c_switched_model_interface/misc/SplineCpg.h>
#include <c_switched_model_interface/misc/CpgBase.h>
#include <c_switched_model_interface/misc/FeetZDirectionPlanner.h>
#include <c_switched_model_interface/misc/SphericalCoordinate.h>
#include <c_switched_model_interface/state_constraint/EndEffectorConstraintBase.h>
#include <c_switched_model_interface/state_constraint/EllipticalConstraint.h>
#include <c_switched_model_interface/state_constraint/EndEffectorConstraintsUtilities.h>
#include <c_switched_model_interface/misc/WeightCompensationForces.h>

#include <ocs2_anymal_switched_model/AnymalSwitchedModel.h>
#include <ocs2_anymal_switched_model/SwitchedModelStateEstimator.h>
#include <ocs2_anymal_switched_model/dynamics/AnymalComKinoDynamics.h>
#include <ocs2_anymal_switched_model/dynamics/AnymalComKinoDynamicsDerivative.h>
#include <iit/robots/anymal/inverse_dynamics.h>
#include <ocs2_anymal_cost/SwitchedModelCost.h>

namespace anymal {

class OCS2AnymalInterface
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<OCS2AnymalInterface> Ptr;
	typedef std::array<Eigen::Vector3d, 4>      contact_array_t;
	typedef ocs2::Dimensions<12+12,12+12> 		dimension_t;
	typedef SwitchedModelCost             		cost_funtion_t;
	typedef AnymalComKinoDynamics           			system_dynamics_t;
	typedef AnymalComKinoDynamicsDerivative 			system_dynamics_derivative_t;
	typedef switched_model::WeightCompensationForces<AnymalCom,AnymalKinematics> weightCompensationForces_t;


	typedef ocs2::GLQP<dimension_t::STATE_DIM_,dimension_t::INPUT_DIM_> 		glqp_t;
	typedef ocs2::SLQP_BASE<dimension_t::STATE_DIM_,dimension_t::INPUT_DIM_> 	slqp_base_t;
	typedef ocs2::SLQP<dimension_t::STATE_DIM_,dimension_t::INPUT_DIM_>  		slqp_t;
	typedef ocs2::SLQP_MP<dimension_t::STATE_DIM_,dimension_t::INPUT_DIM_>  	slqp_mp_t;
	// typedef ocs2::MPC_SLQP<dimension_t::STATE_DIM_,dimension_t::INPUT_DIM_>		mpc_t;
	typedef ocs2::OCS2Projected<dimension_t::STATE_DIM_,dimension_t::INPUT_DIM_> ocs2_t;
	typedef typename glqp_t::Ptr 		glqp_ptr_t;
	typedef typename slqp_base_t::Ptr 	slqp_base_ptr_t;
	typedef typename slqp_t::Ptr  		slqp_ptr_t;
	typedef typename slqp_mp_t::Ptr  	slqp_mp_ptr_t;
	typedef typename ocs2_t::Ptr 		ocs2_ptr_t;
	// typedef typename mpc_t::Ptr 		mpc_ptr_t;
	typedef switched_model::SplineCpg	z_direction_cpg_t;
	typedef Eigen::Matrix<double,6,1>   com_coordinate_t;

	OCS2AnymalInterface(const std::string& pathToConfigFolder, const Eigen::Matrix<double,36,1>& initState)
	: inverseDynamics_(iitInertiaProperties_, iitMotionTransforms_)
	{
		loadSettings(pathToConfigFolder, initState);
		setupOptimizer();

		// LQP
		lqpPtr_ = typename glqp_t::Ptr(new glqp_t(subsystemDynamicsPtr_, subsystemDerivativesPtr_, subsystemCostFunctionsPtr_,
				stateOperatingPoints_, inputOperatingPoints_) );
		// SLQ
		if (slqpOptions_.useMultiThreading_==true)
			slqpPtr_ = slqp_base_ptr_t(new slqp_mp_t(subsystemDynamicsPtr_, subsystemDerivativesPtr_, subsystemCostFunctionsPtr_, slqpOptions_,
					stateOperatingPoints_, inputOperatingPoints_) );
		else
			slqpPtr_ = slqp_base_ptr_t(new slqp_t(subsystemDynamicsPtr_, subsystemDerivativesPtr_, subsystemCostFunctionsPtr_, slqpOptions_,
					stateOperatingPoints_, inputOperatingPoints_) );
		// OCS2
		ocs2Ptr_ = ocs2_ptr_t(new ocs2_t(subsystemDynamicsPtr_, subsystemDerivativesPtr_, subsystemCostFunctionsPtr_, slqpOptions_,
				stateOperatingPoints_, inputOperatingPoints_) );

		// // MPC
		// mpcPtr_ = mpc_ptr_t( new mpc_t(subsystemDynamicsPtr_, subsystemDerivativesPtr_, subsystemCostFunctionsPtr_,
		// 		stateOperatingPoints_, inputOperatingPoints_,
		// 		initSystemStockIndexes_, initSwitchingTimes_,
		// 		slqpOptions_, mpcOptions_) );

	}

	~OCS2AnymalInterface() {}

	void runSLQP(const double& initTime, const Eigen::Matrix<double,36,1>& initState,
			const dimension_t::controller_array_t& initialControllersStock=dimension_t::controller_array_t(),
			const std::vector<double>& switchingTimes=std::vector<double>());

	// bool runMPC(const double& initTime, const Eigen::Matrix<double,36,1>& initState);
	// bool runMPC(const double& initTime, const dimension_t::state_vector_t& initState);

	void runOCS2(const double& initTime, const Eigen::Matrix<double,36,1>& initState,
			const std::vector<double>& switchingTimes=std::vector<double>());

	// void setNewGoalStateMPC(const dimension_t::scalar_t& newGoalDuration, const dimension_t::state_vector_t& newGoalState);

	void getCostFuntion(double& costFunction, double& constriantISE) const;

	void getSwitchingTimes(std::vector<double>& switchingTimes) const;

	void getController(dimension_t::controller_array_t& controllersStockPtr) const;

	std::shared_ptr<const dimension_t::controller_array_t> getControllerPtr() const;

	void getTrajectories(std::vector<dimension_t::scalar_array_t>& timeTrajectoriesStock,
			dimension_t::state_vector_array2_t& stateTrajectoriesStock,
			dimension_t::control_vector_array2_t& inputTrajectoriesStock) const;

	std::shared_ptr<const std::vector<dimension_t::scalar_array_t>> getTimeTrajectoriesPtr() const;

	std::shared_ptr<const dimension_t::state_vector_array2_t> getStateTrajectoriesPtr() const;

	std::shared_ptr<const dimension_t::control_vector_array2_t> getInputTrajectoriesPtr() const;

	void getStanceLegSequene(std::vector<std::array<bool,4> >& stanceLegSequene) const;

	void getSwitchingModeSequence(std::vector<size_t>& switchingModes) const;

	void getSystemStockIndexes(std::vector<size_t>& systemStockIndexes) const;

	void getGapIndicatorPtrs(std::vector<switched_model::EndEffectorConstraintBase::Ptr>& gapIndicatorPtrs) const;

	void getIterationsLog(dimension_t::eigen_scalar_array_t& iterationCost, dimension_t::eigen_scalar_array_t& iterationISE1,
			dimension_t::eigen_scalar_array_t& ocs2Iterationcost) const {
		iterationCost = iterationCost_;
		iterationISE1 = iterationISE1_;
		ocs2Iterationcost = ocs2Iterationcost_;
	}

	void getGravity(Eigen::Vector3d& gravity) { gravity = gravity_; }

	void adjustFootZdirection(const double& time,
			std::array<Eigen::Vector3d,4>& origin_base2StanceFeet, std::array<bool,4>& stanceLegSequene);

	void fromHyqStateToComStateOrigin(const double& time,
			const Eigen::Matrix<double,36,1>& hyqState,
			com_coordinate_t& comPose,
			com_coordinate_t& comVelocities);

	// void getMpcOptions(typename mpc_t::mpc_settings_t& mpcOptions);

	double strideTime() { return (initSwitchingTimes_[1]-initSwitchingTimes_[0]); }

	double strideLength() { return options_.mpcStrideLength_; }

	size_t numPhasesInfullGaitCycle() { return options_.numPhasesInfullGaitCycle_; }


protected:
	void loadSettings(const std::string& pathToConfigFolder, const Eigen::Matrix<double,36,1>& initState);

	void setupOptimizer();

	template<class T>
	void getOptimizerParameters(const std::shared_ptr<T>& optimizerPtr);

	void concatenate();

	size_t findActiveSubsystemIndex(const double& time)  {
		size_t activeSubsystemIndexInStock = numSubsystems_;
		for (size_t i=0; i<numSubsystems_; i++)
			if (time >= switchingTimes_[i])
				activeSubsystemIndexInStock = i;
			else
				break;
		return activeSubsystemIndexInStock;
	}

	void calculateStateDerivative(const std::vector<size_t>& systemStockIndexes,
			const std::vector<double>& switchingTimes,
			const std::vector<dimension_t::scalar_array_t>& timeTrajectoriesStock,
			const dimension_t::state_vector_array2_t& stateTrajectoriesStock,
			const dimension_t::control_vector_array2_t& inputTrajectoriesStock,
			dimension_t::state_vector_array2_t& stateDerivativeTrajectoriesStock);

private:
	size_t initNumSubsystems_;
	size_t numSubsystems_;

	std::array<Eigen::Vector3d,4> origin_base2StanceFeetPrev_;

	iit::ANYmal::dyn::InertiaProperties iitInertiaProperties_;
	iit::ANYmal::MotionTransforms iitMotionTransforms_;
	iit::ANYmal::dyn::InverseDynamics inverseDynamics_;
	size_t greatestLessTimeStampIndex_;

	Eigen::Vector3d gravity_;

	dimension_t::state_matrix_t 	Q_;
	dimension_t::control_matrix_t 	R_;
	dimension_t::state_matrix_t 	QFinal_;
	dimension_t::state_vector_t 	xFinal_;

	double zmpWeight_;
	double impulseWeight_;
	double impulseSigmeFactor_;

	dimension_t::Options slqpOptions_;
	// typename mpc_t::mpc_settings_t mpcOptions_;

	anymal::Options options_;
	std::vector<switched_model::EndEffectorConstraintBase::Ptr> gapIndicatorPtrs_;
	switched_model::FeetZDirectionPlanner<z_direction_cpg_t>::Ptr feetZDirectionPlannerPtr_;
	std::vector<switched_model::CpgBase::PtrArray> plannedCPGs_;

	dimension_t::state_vector_t initSwitchedState_;
	double initTime_;

	std::vector<size_t> 			initSwitchingModes_;
	std::vector<std::array<bool,4>> initStanceLegSequene_;
	std::vector<size_t> 			initSystemStockIndexes_;
	std::vector<double> 			initSwitchingTimes_;

	std::vector<size_t> 			switchingModes_;
	std::vector<std::array<bool,4>> stanceLegSequene_;
	std::vector<size_t> 			systemStockIndexes_;
	std::vector<double> 			switchingTimes_;

	// subsystem dynamics
	std::vector<std::shared_ptr<system_dynamics_t::Base::Base> > subsystemDynamicsPtr_;
	// subsystem derivatives
	std::vector<std::shared_ptr<system_dynamics_derivative_t::Base::Base> > subsystemDerivativesPtr_;
	// subsystem cost functions
	std::vector<std::shared_ptr<cost_funtion_t::BASE> > subsystemCostFunctionsPtr_;
	// subsystem state operating points
	dimension_t::state_vector_array_t   stateOperatingPoints_;
	// subsystem input operating points
	dimension_t::control_vector_array_t inputOperatingPoints_;

	// LQP
	glqp_ptr_t 		lqpPtr_;
	// SLQ
	slqp_base_ptr_t	slqpPtr_;
//	// OCS2
	ocs2_ptr_t 		ocs2Ptr_;
	// // MPC
	// mpc_ptr_t 		mpcPtr_;

	double costFunction_, constriantISE_;
	dimension_t::controller_array_t controllersStock_;
	std::shared_ptr<const dimension_t::controller_array_t> controllersStockPtr_;
	std::shared_ptr<const std::vector<dimension_t::scalar_array_t>> timeTrajectoriesStockPtr_;
	std::shared_ptr<const dimension_t::state_vector_array2_t>   stateTrajectoriesStockPtr_;
	std::shared_ptr<const dimension_t::control_vector_array2_t> inputTrajectoriesStockPtr_;
	std::vector<dimension_t::scalar_array_t> timeTrajectoriesStock_;
	dimension_t::state_vector_array2_t   stateTrajectoriesStock_;
	dimension_t::control_vector_array2_t inputTrajectoriesStock_;
	dimension_t::control_vector_array2_t outputTrajectoriesStock_;
	dimension_t::state_vector_array2_t   stateDerivativeTrajectoriesStock_;

	std::vector<dimension_t::scalar_array_t> 	desiredTimeTrajectoriesStock_;
	dimension_t::state_vector_array2_t  		desiredStateTrajectoriesStock_;

	dimension_t::eigen_scalar_array_t iterationCost_;
	dimension_t::eigen_scalar_array_t iterationISE1_;
	dimension_t::eigen_scalar_array_t ocs2Iterationcost_;

	dimension_t::scalar_array_t 		timeTrajectory_;
	dimension_t::state_vector_array_t 	stateTrajectory_;
	dimension_t::control_vector_array_t inputTrajectory_;
	dimension_t::state_vector_array_t 	stateDerivativeTrajectory_;

	dimension_t::scalar_array_t 			controllerTimeTrajectory_;
	dimension_t::control_feedback_array_t 	controllerFBTrajectory_;
	dimension_t::control_vector_array_t   	controllerFFTrajector_;

	ocs2::LinearInterpolation<dimension_t::state_vector_t, Eigen::aligned_allocator<dimension_t::state_vector_t> > 			linInterpolateState_;
	ocs2::LinearInterpolation<dimension_t::control_vector_t, Eigen::aligned_allocator<dimension_t::control_vector_t> > 		linInterpolateInput_;
	ocs2::LinearInterpolation<dimension_t::state_vector_t, Eigen::aligned_allocator<dimension_t::state_vector_t> > 			linInterpolateStateDerivative_;
	ocs2::LinearInterpolation<dimension_t::control_vector_t, Eigen::aligned_allocator<dimension_t::control_vector_t> > 		linInterpolateUff_;
	ocs2::LinearInterpolation<dimension_t::control_feedback_t, Eigen::aligned_allocator<dimension_t::control_feedback_t> > 	linInterpolateK_;

	dimension_t::state_vector_t mrtDeadzone_;

	const Eigen::IOFormat CleanFmtDisplay_ = Eigen::IOFormat(3, 0, ", ", "\n", "[", "]");

	weightCompensationForces_t weightCompensationForces_;

};

} // end of namespace anymal

#endif /* OCS2ANYMALINTERFACE_H_ */
