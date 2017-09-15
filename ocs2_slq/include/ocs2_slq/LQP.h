/*
 * LQP.h
 *
 *  Created on: Aug 3, 2016
 *      Author: farbod
 */

#ifndef LQP_OCS2_H_
#define LQP_OCS2_H_

#include <vector>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/cost/CostFunctionBaseOCS2.h>
#include <ocs2_core/integration/Integrator.h>

#include "ocs2_slq/SolveBVP.h"

namespace ocs2{

/**
 * This class implements the LQ method. The method is equivalent to SLQ. However, the nominal trajectories
 * are fixed and given in contrast to SLQ which derives them by forward integrating the system dynamics.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam NUM_Subsystems
 */
template <size_t STATE_DIM, size_t INPUT_DIM, size_t NUM_Subsystems>
class LQP
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	const bool INFO_ON_ = false;
	//
	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::controller_t controller_t;
	typedef typename DIMENSIONS::controller_array_t controller_array_t;
	typedef typename DIMENSIONS::Options Options_t;
	typedef typename DIMENSIONS::scalar_t 		 scalar_t;
	typedef typename DIMENSIONS::scalar_array_t  scalar_array_t;
	typedef typename DIMENSIONS::eigen_scalar_t       eigen_scalar_t;
	typedef typename DIMENSIONS::eigen_scalar_array_t eigen_scalar_array_t;
	typedef typename DIMENSIONS::state_vector_t 	  state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::state_vector_array2_t state_vector_array2_t;
	typedef typename DIMENSIONS::control_vector_t 		control_vector_t;
	typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
	typedef typename DIMENSIONS::control_vector_array2_t control_vector_array2_t;
	typedef typename DIMENSIONS::control_feedback_t 	  control_feedback_t;
	typedef typename DIMENSIONS::control_feedback_array_t control_feedback_array_t;
	typedef typename DIMENSIONS::state_matrix_t 	  state_matrix_t;
	typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
	typedef typename DIMENSIONS::state_matrix_array2_t state_matrix_array2_t;
	typedef typename DIMENSIONS::control_matrix_t 		control_matrix_t;
	typedef typename DIMENSIONS::control_matrix_array_t control_matrix_array_t;
	typedef typename DIMENSIONS::control_gain_matrix_t 		 control_gain_matrix_t;
	typedef typename DIMENSIONS::control_gain_matrix_array_t control_gain_matrix_array_t;

	/**
	 *
	 * @param [in] subsystemDynamicsPtr: Array of system dynamics and constraints for system's subsystems.
	 * @param [in] subsystemDerivativesPtr: Array of system dynamics and constraints dervatives for system's subsystems.
	 * @param [in] subsystemCostFunctionsPtr: Array of cost function and its dervatives for system's subsystems.
	 * @param [in] stateOperatingPoints: The state operating points for system's subsystems which will be used for initialization of LQ.
	 * @param [in] inputOperatingPoints: The input operating points for system's subsystems which will be used for initialization of LQ.
	 * @param [in] systemStockIndexes: The indexes of the susbsystms in subsystemDynamicsPtr, subsystemDerivativesPtr, or subsystemCostFunctionsPtr.
	 * @param [in] options: Structure containing the settings for the LQ algorithm.
	 * @param [in] runAsInitializer: Flag to determine to run the algorithm as an initializer.
	 */
	LQP(const std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> > >& subsystemDynamicsPtr,
			const std::vector<std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM> > >& subsystemDerivativesPtr,
			const std::vector<std::shared_ptr<CostFunctionBaseOCS2<STATE_DIM, INPUT_DIM> > >& subsystemCostFunctionsPtr,
			const state_vector_array_t&   stateOperatingPoints,
			const control_vector_array_t& inputOperatingPoints,
			const std::vector<size_t>& systemStockIndexes,
			const Options_t& options = Options_t::Options(),
			const bool& runAsInitializer = true)

		: subsystemDynamicsPtrStock_(NUM_Subsystems),
		  subsystemDerivativesPtrStock_(NUM_Subsystems),
		  subsystemCostFunctionsPtrStock_(NUM_Subsystems),
		  stateOperatingPointsStock_(NUM_Subsystems),
		  inputOperatingPointsStock_(NUM_Subsystems),
		  systemStockIndexes_(systemStockIndexes),
		  options_(options),
		  runAsInitializer_(runAsInitializer),
		  subsystemSimulatorsStockPtr_(NUM_Subsystems),
		  controllersStock_(NUM_Subsystems),
		  AmStock_(NUM_Subsystems),
		  BmStock_(NUM_Subsystems),
		  GvStock_(NUM_Subsystems),
		  QvStock_(NUM_Subsystems),
		  QmStock_(NUM_Subsystems),
		  RvStock_(NUM_Subsystems),
		  RmStock_(NUM_Subsystems),
		  RmInverseStock_(NUM_Subsystems),
		  PmStock_(NUM_Subsystems),
		  timeTrajectoryStock_(NUM_Subsystems),
		  SvTrajectoryStock_(NUM_Subsystems),
		  SmTrajectoryStock_(NUM_Subsystems),
		  switchingTimes_(NUM_Subsystems+1)
	{

		if (subsystemDynamicsPtr.size() != subsystemDerivativesPtr.size())
			throw std::runtime_error("Number of subsystem derivaties is not equal to the number of subsystems.");
		if (subsystemDynamicsPtr.size() != subsystemCostFunctionsPtr.size())
			throw std::runtime_error("Number of cost functions is not equal to the number of subsystems.");
		if (subsystemDynamicsPtr.size() != stateOperatingPoints.size())
			throw std::runtime_error("Number of state operating points is not equal to the number of subsystems.");
		if (subsystemDynamicsPtr.size() != inputOperatingPoints.size())
			throw std::runtime_error("Number of input operating points is not equal to the number of subsystems.");
		if (subsystemDynamicsPtr.size()-1 < *std::max_element(systemStockIndexes.begin(), systemStockIndexes.end()))
			throw std::runtime_error("systemStockIndexes points to non-existing subsystem");
		if (systemStockIndexes.size() != NUM_Subsystems)
			throw std::runtime_error("systemStockIndexes has less elements then the number of subsystems");

		for (int i=0; i<NUM_Subsystems; i++) {

			subsystemDynamicsPtrStock_[i] = subsystemDynamicsPtr[systemStockIndexes[i]]->clone();
			subsystemDerivativesPtrStock_[i] = subsystemDerivativesPtr[systemStockIndexes[i]]->clone();
			subsystemCostFunctionsPtrStock_[i] = subsystemCostFunctionsPtr[systemStockIndexes[i]]->clone();

			stateOperatingPointsStock_[i] = stateOperatingPoints[systemStockIndexes[i]];
			inputOperatingPointsStock_[i] = inputOperatingPoints[systemStockIndexes[i]];

			subsystemSimulatorsStockPtr_[i] = std::shared_ptr<ODE45<STATE_DIM>>( new ODE45<STATE_DIM>(subsystemDynamicsPtrStock_[i]) );
		}
	}

	/**
	 * Default destructor.
	 */
	~LQP() {}

	/**
	 * Forward integrate the system dynamics with given controller. It uses the given control policies and initial state,
	 * to integrate the system dyanmics in time period [initTime, finalTime].
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
	 * @param [in] timeTrajectoriesStock: Array of trajectories containing the time trajectory stamp of a rollout.
	 * @param [in] stateTrajectoriesStock: Array of trajectories containing the state trajectory of a rollout.
	 * @param [in] inputTrajectoriesStock: Array of trajectories containing the control input trajectory of a rollout.
	 * @param [out] totalCost: The total cost of the rollout.
	 */
	void rolloutCost(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const control_vector_array2_t& inputTrajectoriesStock,
			scalar_t& totalCost);

	/**
	 * Gets the optimal array of the control policies.
	 *
	 * @param [out] controllersStock: The optimal array of the control policies.
	 */
	void getController(controller_array_t& controllersStock);

	/**
	 * The main routine of LQ which runs LQP for a given state, time, subsystem arrangement, and switching times.

	 * @param [in] switchingTimes: The switching times between subsystems.
	 * @param [in] learningRate: Learning rate for the line search.
	 */
	void run(const std::vector<scalar_t>& switchingTimes, const scalar_t& learningRate=1.0);

protected:
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
	 * Makes the matrix PSD.
	 * @tparam Derived type.
	 * @param [out] squareMatrix: The matrix to become PSD.
	 * @return boolean
	 */
	template <typename Derived>
	bool makePSD(Eigen::MatrixBase<Derived>& squareMatrix);

private:
	std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> > > subsystemDynamicsPtrStock_;
	std::vector<std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM> > > subsystemDerivativesPtrStock_;
	std::vector<std::shared_ptr<CostFunctionBaseOCS2<STATE_DIM, INPUT_DIM> > > subsystemCostFunctionsPtrStock_;

	state_vector_array_t   stateOperatingPointsStock_;
	control_vector_array_t inputOperatingPointsStock_;

	std::vector<size_t> systemStockIndexes_;

	Options_t options_;

	bool runAsInitializer_;

	std::vector<std::shared_ptr<ODE45<STATE_DIM> > > subsystemSimulatorsStockPtr_;

	controller_array_t controllersStock_;

	state_matrix_array_t        AmStock_;
	control_gain_matrix_array_t BmStock_;
	state_vector_array_t       GvStock_;

	state_vector_t QvFinal_;
	state_matrix_t QmFinal_;
	state_vector_array_t QvStock_;
	state_matrix_array_t QmStock_;
	control_vector_array_t RvStock_;
	control_matrix_array_t RmStock_;
	control_matrix_array_t RmInverseStock_;
	control_feedback_array_t PmStock_;

	std::vector<scalar_array_t> timeTrajectoryStock_;
	state_vector_array2_t  SvTrajectoryStock_;
	state_matrix_array2_t  SmTrajectoryStock_;

	scalar_array_t switchingTimes_;
};

} // namespace ocs2

#include "implementation/LQP.h"


#endif /* LQP_OCS2_H_ */
