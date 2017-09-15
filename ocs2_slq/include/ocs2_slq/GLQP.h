/*
 * GLQP.h
 *
 *  Created on: Jan 5, 2016
 *      Author: farbod
 */

#ifndef GLQP_OCS2_H_
#define GLQP_OCS2_H_

#include <iostream>
#include <vector>
#include <memory>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/cost/CostFunctionBaseOCS2.h>
#include <ocs2_core/integration/Integrator.h>
#include <ocs2_core/misc/LinearInterpolation.h>

#include "ocs2_slq/PartialRiccatiEquations.h"


namespace ocs2{

/**
 * This class implements the LQ method. The method is equivalent to SLQ. However, the nominal trajectories
 * are fixed and given in contrast to SLQ which derives them by forward integrating the system dynamics.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class GLQP
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	const bool INFO_ON_ = false;
	typedef std::shared_ptr<GLQP<STATE_DIM, INPUT_DIM> > Ptr;

	typedef PartialRiccatiEquations<STATE_DIM, INPUT_DIM> RiccatiEquations_t;
	//
	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::controller_t controller_t;
	typedef typename DIMENSIONS::controller_array_t controller_array_t;
	typedef typename DIMENSIONS::scalar_t 		scalar_t;
	typedef typename DIMENSIONS::eigen_scalar_t       eigen_scalar_t;
	typedef typename DIMENSIONS::eigen_scalar_array_t eigen_scalar_array_t;
	typedef typename DIMENSIONS::eigen_scalar_array2_t eigen_scalar_array2_t;
	typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
	typedef typename DIMENSIONS::state_vector_t 	  state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t  state_vector_array_t;
	typedef typename DIMENSIONS::state_vector_array2_t state_vector_array2_t;
	typedef typename DIMENSIONS::control_vector_t 		control_vector_t;
	typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
	typedef typename DIMENSIONS::control_vector_array2_t control_vector_array2_t;
	typedef typename DIMENSIONS::control_feedback_t 	  control_feedback_t;
	typedef typename DIMENSIONS::control_feedback_array_t control_feedback_array_t;
	typedef typename DIMENSIONS::state_matrix_t 	   state_matrix_t;
	typedef typename DIMENSIONS::state_matrix_array_t  state_matrix_array_t;
	typedef typename DIMENSIONS::state_matrix_array2_t state_matrix_array2_t;
	typedef typename DIMENSIONS::control_matrix_t 		control_matrix_t;
	typedef typename DIMENSIONS::control_matrix_array_t control_matrix_array_t;
	typedef typename DIMENSIONS::control_gain_matrix_t 		 control_gain_matrix_t;
	typedef typename DIMENSIONS::control_gain_matrix_array_t control_gain_matrix_array_t;

	/**
	 * Constructor
	 *
	 * @param [in] subsystemDynamicsPtr: Array of system dynamics and constraints for system's subsystems.
	 * @param [in] subsystemDerivativesPtr: Array of system dynamics and constraints derivatives for system's subsystems.
	 * @param [in] subsystemCostFunctionsPtr: Array of cost function and its derivatives for system's subsystems.
	 * @param [in] stateOperatingPoints: The state operating points for system's subsystems which will be used for initialization of LQ.
	 * @param [in] inputOperatingPoints: The input operating points for system's subsystems which will be used for initialization of LQ.
	 */
	GLQP(const std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> > >& subsystemDynamicsPtr,
			const std::vector<std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM> > >& subsystemDerivativesPtr,
			const std::vector<std::shared_ptr<CostFunctionBaseOCS2<STATE_DIM, INPUT_DIM> > >& subsystemCostFunctionsPtr,
			const state_vector_array_t&   stateOperatingPoints,
			const control_vector_array_t& inputOperatingPoints)
	: numSubsystems_(0),
	  subsystemDynamicsPtr_(subsystemDynamicsPtr),
	  subsystemDerivativesPtr_(subsystemDerivativesPtr),
	  subsystemCostFunctionsPtr_(subsystemCostFunctionsPtr),
	  stateOperatingPoints_(stateOperatingPoints),
	  inputOperatingPoints_(inputOperatingPoints)
	{
		if (subsystemDynamicsPtr.size() != subsystemDerivativesPtr.size())
			throw std::runtime_error("Number of subsystem derivaties is not equal to the number of subsystems.");
		if (subsystemDynamicsPtr.size() != subsystemCostFunctionsPtr.size())
			throw std::runtime_error("Number of cost functions is not equal to the number of subsystems.");
		if (subsystemDynamicsPtr.size() != stateOperatingPoints.size())
			throw std::runtime_error("Number of state operating points is not equal to the number of subsystems.");
		if (subsystemDynamicsPtr.size() != inputOperatingPoints.size())
			throw std::runtime_error("Number of input operating points is not equal to the number of subsystems.");
	}

	/**
	 * Default destructor.
	 */
	~GLQP() {}

	/**
	 * Forward integrate the system dynamics with given controller. It uses the given control policies and initial state,
	 * to integrate the system dynamics in time period [initTime, finalTime].
	 *
	 * @param [in] initState: Initial state.
	 * @param [in] controllersStock: Array of control policies.
	 * @param [out] timeTrajectoriesStock: Array of trajectories containing the output time trajectory stamp.
	 * @param [out] stateTrajectoriesStock: Array of trajectories containing the output state trajectory.
	 * @param [out] inputTrajectoriesStock: Array of trajectories containing the output control input trajectory.
	 */
	void rollout(const state_vector_t& initState,
			const controller_array_t& controllersStock,
			std::vector<scalar_array_t>& timeTrajectoriesStock,
			state_vector_array2_t& stateTrajectoriesStock,
			control_vector_array2_t& inputTrajectoriesStock);

	/**
	 * Calculates cost of a rollout.
	 *
	 * @param [in] timeTrajectoriesStock: Array of trajectories containing the time trajectory stamp of a roll-out.
	 * @param [in] stateTrajectoriesStock: Array of trajectories containing the state trajectory of a roll-out.
	 * @param [in] controlTrajectoriesStock: Array of trajectories containing the control input trajectory of a roll-out.
	 * @param [out] totalCost: The total cost of the roll-out.
	 */
	void rolloutCost(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const control_vector_array2_t& controlTrajectoriesStock,
			scalar_t& totalCost);

	/**
	 * Gets the optimal array of the control policies.
	 *
	 * @param [out] controllersStock: The optimal array of the control policies.
	 */
	void getController(controller_array_t& controllersStock);

	/**
	 * Calculates the value function at the given time and state.
	 *
	 * @param [in] time: The inquiry time
	 * @param [in] state: The inquiry state.
	 * @param [out] valueFuntion: value function at the inquiry time and state.
	 */
	void getValueFuntion(const scalar_t& time, const state_vector_t& state, scalar_t& valueFuntion);

	/**
	 * The main routine of LQ which runs GLQP for a given state, time, subsystem arrangement, and switching times.

	 * @param [in] systemStockIndexes: The indexes of the subsystems in subsystemDynamicsPtr, subsystemDerivativesPtr, or subsystemCostFunctionsPtr.
	 * @param [in] switchingTimes: The switching times between subsystems.
	 * @param [in] learningRate: Learning rate for the line search.
	 * @param [in] desiredTimeTrajectoriesStock: The time stamp trajectory for each subsystem's cost.
	 * @param [in] desiredStateTrajectoriesStock: The state trajectory for each subsystem's cost.
	 */
	void run(const std::vector<size_t>& systemStockIndexes,
		const std::vector<scalar_t>& switchingTimes, const scalar_t& learningRate=1.0,
		const std::vector<scalar_array_t>& desiredTimeTrajectoriesStock = std::vector<scalar_array_t>(),
		const state_vector_array2_t& desiredStateTrajectoriesStock = state_vector_array2_t());

	/**
	 * Sets nominal state of subsystem costs.
	 *
	 * @param [in] timeTrajectoryStock: The time stamp trajectory for each subsystem's cost.
	 * @param [in] stateTrajectoryStock: The state trajectory for each subsystem's cost.
	 */
	virtual void setCostNominalStates(const std::vector<scalar_array_t>& timeTrajectoryStock,
			const state_vector_array2_t& stateTrajectoryStock);

protected:
	/**
	 * Sets up uptomizer
	 */
	void setupOptimizer();

    /**
     * SOlves Riccati equations
     */
	void SolveRiccatiEquations();

    /**
     * Approximates optimal control problem
     */
	void approximateOptimalControlProblem();

	/**
	 * Calculates the controller.
	 *
	 * @param [in] learningRate: Learning rate.
	 * @param [out] controllersStock: the output array of control policies.
	 */
	void calculateController(const scalar_t& learningRate, controller_array_t& controllersStock);

    /**
     * Transforms local value function to global
     */
	void transformeLocalValueFuntion2Global();

	/**
	 * Makes the matrix PSD.
	 * @tparam Derived type.
	 * @param [out] squareMatrix: The matrix to become PSD.
	 * @return boolean
	 */
	template <typename Derived>
	bool makePSD(Eigen::MatrixBase<Derived>& squareMatrix);

private:
	size_t numSubsystems_;
	std::vector<size_t> systemStockIndexes_;
	scalar_array_t switchingTimes_;

	std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> > > subsystemDynamicsPtr_;
	std::vector<std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM> > > subsystemDerivativesPtr_;
	std::vector<std::shared_ptr<CostFunctionBaseOCS2<STATE_DIM, INPUT_DIM> > > subsystemCostFunctionsPtr_;
	state_vector_array_t   stateOperatingPoints_;
	control_vector_array_t inputOperatingPoints_;

	std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> > > subsystemDynamicsPtrStock_;
	std::vector<std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM> > > subsystemDerivativesPtrStock_;
	std::vector<std::shared_ptr<CostFunctionBaseOCS2<STATE_DIM, INPUT_DIM> > > subsystemCostFunctionsPtrStock_;

	state_vector_array_t   stateOperatingPointsStock_;
	control_vector_array_t inputOperatingPointsStock_;

	std::vector<std::shared_ptr<ODE45<STATE_DIM> > > subsystemSimulatorsStockPtr_;

	controller_array_t controllersStock_;

	std::vector<scalar_array_t> desiredTimeTrajectoriesStock_;
	state_vector_array2_t   	desiredStateTrajectoriesStock_;

	state_matrix_array_t        AmStock_;
	control_gain_matrix_array_t BmStock_;

	eigen_scalar_t  qFinal_;
	state_vector_t  QvFinal_;
	state_matrix_t  QmFinal_;
	eigen_scalar_array_t  qStock_;
	state_vector_array_t QvStock_;
	state_matrix_array_t  QmStock_;
	control_vector_array_t   RvStock_;
	control_matrix_array_t   RmStock_;
	control_feedback_array_t PmStock_;

	std::vector<scalar_array_t> timeTrajectoryStock_;
	eigen_scalar_array2_t  		sTrajectoryStock_;
	state_vector_array2_t 		SvTrajectoryStock_;
	state_matrix_array2_t  		SmTrajectoryStock_;
};

} // namespace ocs2

#include "implementation/GLQP.h"

#endif /* GLQP_H_ */
