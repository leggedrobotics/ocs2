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

	~GLQP() {}

	void rollout(const state_vector_t& initState,
			const controller_array_t& controllersStock,
			std::vector<scalar_array_t>& timeTrajectoriesStock,
			state_vector_array2_t& stateTrajectoriesStock,
			control_vector_array2_t& inputTrajectoriesStock);

	void rolloutCost(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const control_vector_array2_t& controlTrajectoriesStock,
			scalar_t& totalCost);

	void getController(controller_array_t& controllersStock);

	void getValueFuntion(const scalar_t& time, const state_vector_t& state, scalar_t& valueFuntion);

	void run(const std::vector<size_t>& systemStockIndexes,
		const std::vector<scalar_t>& switchingTimes, const scalar_t& learningRate=1.0,
		const std::vector<scalar_array_t>& desiredTimeTrajectoriesStock = std::vector<scalar_array_t>(),
		const state_vector_array2_t& desiredStateTrajectoriesStock = state_vector_array2_t());

	virtual void setCostNominalStates(const std::vector<scalar_array_t>& timeTrajectoryStock,
			const state_vector_array2_t& stateTrajectoryStock);

protected:
	void setupOptimizer();

	void SolveRiccatiEquations();

	void approximateOptimalControlProblem();

	void calculatecontroller(const scalar_t& learningRate, controller_array_t& controllersStock);

	void transformeLocalValueFuntion2Global();

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
