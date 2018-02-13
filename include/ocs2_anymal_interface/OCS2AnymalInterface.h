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
#include <csignal>
#include <chrono>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include<Eigen/StdVector>
#include <vector>

#include <ocs2_slq/SLQ.h>
#include <ocs2_slq/SLQ_MP.h>
//#include <ocs2_ocs2/OCS2Projected.h>
//#include <ocs2_core/misc/LinearInterpolation.h>

#include <ocs2_mpc/MPC_Settings.h>
//#include <ocs2_mpc/MPC_SLQ.h>

#include <c_switched_model_interface/core/Model_Settings.h>
#include <c_switched_model_interface/core/MotionPhaseDefinition.h>
#include <c_switched_model_interface/misc/SphericalCoordinate.h>
#include <c_switched_model_interface/state_constraint/EndEffectorConstraintBase.h>
#include <c_switched_model_interface/state_constraint/EllipticalConstraint.h>
#include <c_switched_model_interface/state_constraint/EndEffectorConstraintsUtilities.h>

#include <ocs2_anymal_switched_model/logic/AnymalLogicRules.h>
#include <ocs2_anymal_switched_model/core/AnymalModelStateEstimator.h>
#include <ocs2_anymal_switched_model/dynamics/AnymalComKinoDynamics.h>
#include <ocs2_anymal_switched_model/dynamics_derivative/AnymalComKinoDynamicsDerivative.h>
#include <ocs2_anymal_switched_model/constraint/AnymalComKinoConstraint.h>
#include <ocs2_anymal_switched_model/cost/AnymalCost.h>
#include <ocs2_anymal_switched_model/initialization/AnymalComKinoOperatingPoints.h>
#include <ocs2_anymal_switched_model/misc/AnymalWeightCompensationForces.h>
#include <iit/robots/anymal/inverse_dynamics.h>

#include <ocs2_core/misc/LoadConfigFile.h>
#include <ocs2_core/integration/eigenIntegration.h>

namespace anymal {

class OCS2AnymalInterface
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<OCS2AnymalInterface> Ptr;
	typedef std::array<Eigen::Vector3d, 4>      contact_array_t;
	typedef ocs2::Dimensions<12+12,12+12> 		dimension_t;
	typedef switched_model::SwitchedModelLogicRulesBase<12> logic_rules_t;
	typedef AnymalCost		             		cost_funtion_t;
	typedef AnymalComKinoConstraint     		constraint_t;
	typedef AnymalComKinoDynamics           	system_dynamics_t;
	typedef AnymalComKinoDynamicsDerivative 	system_dynamics_derivative_t;
	typedef AnymalComKinoOperatingPoints	 	operating_point_t;
	typedef AnymalWeightCompensationForces		weightCompensationForces_t;

//	const int STATE_DIM = dimension_t::DIMS::STATE_DIM_;
//	const int INPUT_DIM = dimension_t::DIMS::INPUT_DIM_;

	using DIMS = dimension_t::DIMS;

	typedef ocs2::SLQ_BASE<DIMS::STATE_DIM_, DIMS::INPUT_DIM_, logic_rules_t>	slqp_base_t;
	typedef ocs2::SLQ<DIMS::STATE_DIM_, DIMS::INPUT_DIM_, logic_rules_t>  		slqp_t;
	typedef ocs2::SLQ_MP<DIMS::STATE_DIM_, DIMS::INPUT_DIM_, logic_rules_t>  	slqp_mp_t;
//	typedef ocs2::MPC_SLQ<STATE_DIM, INPUT_DIM, logic_rules_t>	mpc_t;
//	typedef ocs2::OCS2Projected<STATE_DIM, INPUT_DIM> 	ocs2_t;
	typedef typename slqp_base_t::Ptr 	slqp_base_ptr_t;
	typedef typename slqp_t::Ptr  		slqp_ptr_t;
	typedef typename slqp_mp_t::Ptr  	slqp_mp_ptr_t;
//	typedef typename ocs2_t::Ptr 		ocs2_ptr_t;
//	typedef typename mpc_t::Ptr 		mpc_ptr_t;

	typedef Eigen::Matrix<double,6,1>   com_coordinate_t;

	OCS2AnymalInterface(const std::string& pathToConfigFolder)
	: inverseDynamics_(iitInertiaProperties_, iitMotionTransforms_)
	{
		loadSettings(pathToConfigFolder);
		setupOptimizer();

		// SLQ
		if (modelSettings_.useMultiThreading_==true) {
			slqPtr_ = slqp_base_ptr_t(new slqp_mp_t(dynamicsPtr_.get(), dynamicsDerivativesPtr_.get(), constraintsPtr_.get(),
					costFunctionPtr_.get(), operatingPointsPtr_.get(), slqSettings_, *logicRulesPtr_) );
		} else {
			slqPtr_ = slqp_base_ptr_t(new slqp_t(dynamicsPtr_.get(), dynamicsDerivativesPtr_.get(), constraintsPtr_.get(),
					costFunctionPtr_.get(), operatingPointsPtr_.get(), slqSettings_, *logicRulesPtr_) );
		}
//		// OCS2
//		ocs2Ptr_ = ocs2_ptr_t(new ocs2_t(subsystemDynamicsPtr_, subsystemDerivativesPtr_, subsystemCostFunctionsPtr_, slqSettings_,
//				stateOperatingPoints_, inputOperatingPoints_) );
//
//		// MPC
//		mpcPtr_ = mpc_ptr_t( new mpc_t(subsystemDynamicsPtr_, subsystemDerivativesPtr_, subsystemCostFunctionsPtr_,
//				stateOperatingPoints_, inputOperatingPoints_,
//				initSystemStockIndexes_, initSwitchingTimes_,
//				slqSettings_, mpcSettings_) );

	}

	~OCS2AnymalInterface() {}

	void runSLQ(const double& initTime, const Eigen::Matrix<double,36,1>& initState,
			const dimension_t::controller_array_t& initialControllersStock=dimension_t::controller_array_t(),
			const std::vector<double>& switchingTimes=std::vector<double>());

	bool runMPC(const double& initTime, const Eigen::Matrix<double,36,1>& initState);

	bool runMPC(const double& initTime, const dimension_t::state_vector_t& initState);

	void runOCS2(const double& initTime, const Eigen::Matrix<double,36,1>& initState,
			const std::vector<double>& switchingTimes=std::vector<double>());

	void setNewGoalStateMPC(const dimension_t::scalar_t& newGoalDuration,
			const dimension_t::state_vector_t& newGoalState);

	void getPerformanceIndeces(double& costFunction, double& constriantISE1, double& constriantISE2) const;

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

	void getGapIndicatorPtrs(std::vector<switched_model::EndEffectorConstraintBase::ConstPtr>& gapIndicatorPtrs) const;

	void getIterationsLog(dimension_t::eigen_scalar_array_t& iterationCost, dimension_t::eigen_scalar_array_t& iterationISE1,
			dimension_t::eigen_scalar_array_t& ocs2Iterationcost) const {
		iterationCost = iterationCost_;
		iterationISE1 = iterationISE1_;
		ocs2Iterationcost = ocs2Iterationcost_;
	}

	void adjustFootZdirection(const double& time,
			std::array<Eigen::Vector3d,4>& origin_base2StanceFeet, std::array<bool,4>& stanceLegSequene);

	void fromHyqStateToComStateOrigin(const double& time,
			const Eigen::Matrix<double,36,1>& hyqState,
			com_coordinate_t& comPose,
			com_coordinate_t& comVelocities);

	void getMpcOptions(ocs2::MPC_Settings& mpcOptions);

	double strideTime() { return (initSwitchingTimes_[1]-initSwitchingTimes_[0]); }

	double strideLength() { return modelSettings_.mpcStrideLength_; }

	size_t numPhasesInfullGaitCycle() { return modelSettings_.numPhasesInfullGaitCycle_; }

	/**
	 * This function loads the simulation-specific settings:
	 * dt, tFinal, initSettlingTime
	 */
	static void loadSimulationSettings(const std::string& filename, double& dt, double& tFinal,
			double& initSettlingTime);

	/**
	 * This function loads the visualization-specific settings:
	 * slowdown factor, vizualization time
	 */
	static void loadVisualizationSettings(const std::string& filename, double& slowdown, double& vizTime);

protected:
	void loadSettings(const std::string& pathToConfigFile);

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

private:

	std::array<Eigen::Vector3d,4> origin_base2StanceFeetPrev_;

	iit::ANYmal::dyn::InertiaProperties iitInertiaProperties_;
	iit::ANYmal::MotionTransforms iitMotionTransforms_;
	iit::ANYmal::dyn::InverseDynamics inverseDynamics_;
	size_t greatestLessTimeStampIndex_;

	dimension_t::state_matrix_t 	Q_;
	dimension_t::control_matrix_t 	R_;
	dimension_t::state_matrix_t 	QFinal_;
	dimension_t::state_vector_t 	xFinal_;

	double impulseWeight_;
	double impulseSigmeFactor_;

	ocs2::SLQ_Settings slqSettings_;
	ocs2::MPC_Settings mpcSettings_;

	switched_model::Model_Settings modelSettings_;

	std::vector<switched_model::EndEffectorConstraintBase::ConstPtr> gapIndicatorPtrs_;

	AnymalModelStateEstimator switchedModelStateEstimator_;

	Eigen::Matrix<double,36,1> 	initRbdState_;
	dimension_t::state_vector_t initSwitchedState_;

	double 				initTime_;
	double 				finalTime_;
	std::vector<double>	partitioningTimes_;

	size_t 							initNumSubsystems_;
	std::vector<double> 			initSwitchingTimes_;
	std::vector<size_t> 			initSwitchingModes_;
	std::vector<std::array<bool,4>> initStanceLegSequene_;

	size_t 							numSubsystems_;
	std::vector<double> 			switchingTimes_;
	std::vector<size_t> 			switchingModes_;
	std::vector<std::array<bool,4>> stanceLegSequene_;

	// logic
	std::shared_ptr<logic_rules_t> 					logicRulesPtr_;
	// dynamics
	std::shared_ptr<system_dynamics_t> 				dynamicsPtr_;
	// dynamics derivatives
	std::shared_ptr<system_dynamics_derivative_t> 	dynamicsDerivativesPtr_;
	// constraints
	std::shared_ptr<constraint_t> 					constraintsPtr_;
	// cost function
	std::shared_ptr<cost_funtion_t> 				costFunctionPtr_;
	// operating points
	std::shared_ptr<operating_point_t> 				operatingPointsPtr_;

	// SLQ
	slqp_base_ptr_t	slqPtr_;
	// OCS2
//	ocs2_ptr_t 		ocs2Ptr_;
	// MPC
//	mpc_ptr_t 		mpcPtr_;

	double costFunction_, constriantISE1_, constriantISE2_;
	dimension_t::controller_array_t controllersStock_;
	std::shared_ptr<const dimension_t::controller_array_t> controllersStockPtr_;
	std::shared_ptr<const std::vector<dimension_t::scalar_array_t>> timeTrajectoriesStockPtr_;
	std::shared_ptr<const dimension_t::state_vector_array2_t>   stateTrajectoriesStockPtr_;
	std::shared_ptr<const dimension_t::control_vector_array2_t> inputTrajectoriesStockPtr_;
	std::vector<dimension_t::scalar_array_t> timeTrajectoriesStock_;
	dimension_t::state_vector_array2_t   stateTrajectoriesStock_;
	dimension_t::control_vector_array2_t inputTrajectoriesStock_;
	dimension_t::control_vector_array2_t outputTrajectoriesStock_;

	std::vector<dimension_t::scalar_array_t> 	desiredTimeTrajectoriesStock_;
	dimension_t::state_vector_array2_t  		desiredStateTrajectoriesStock_;

	dimension_t::eigen_scalar_array_t iterationCost_;
	dimension_t::eigen_scalar_array_t iterationISE1_;
	dimension_t::eigen_scalar_array_t iterationISE2_;
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
	ocs2::LinearInterpolation<dimension_t::control_vector_t, Eigen::aligned_allocator<dimension_t::control_vector_t> > 		linInterpolateUff_;
	ocs2::LinearInterpolation<dimension_t::control_feedback_t, Eigen::aligned_allocator<dimension_t::control_feedback_t> > 	linInterpolateK_;

	const Eigen::IOFormat CleanFmtDisplay_ = Eigen::IOFormat(3, 0, ", ", "\n", "[", "]");

	weightCompensationForces_t weightCompensationForces_;

};

} // end of namespace anymal

#endif /* OCS2ANYMALINTERFACE_H_ */
