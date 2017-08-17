/*
 * GSLQP.h
 *
 *  Created on: Dec 18, 2015
 *      Author: farbod
 */

#ifndef GSLQP_OCS2_H_
#define GSLQP_OCS2_H_

#include <vector>
#include <array>
#include <algorithm>
#include <cstddef>
#include <omp.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/cost/CostFunctionBaseOCS2.h>
#include <ocs2_core/integration/Integrator.h>
#include <ocs2_core/misc/LinearInterpolation.h>

#include <ocs2_slq/GLQP.h>
#include <ocs2_slq/SLQP_BASE.h>
#include <ocs2_slq/SLQP.h>
#include <ocs2_slq/SLQP_MP.h>
#include <ocs2_slq/SolveBVP.h>

#include "ocs2_ocs2/SensitivitySequentialRiccatiEquations.h"
#include "ocs2_ocs2/RolloutSensitivityEquations.h"

namespace ocs2{

template <size_t STATE_DIM, size_t INPUT_DIM>
class GSLQP
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<GSLQP<STATE_DIM, INPUT_DIM> > Ptr;
	typedef SLQP_BASE<STATE_DIM, INPUT_DIM>	slqp_t;
	typedef typename slqp_t::Ptr			slqp_ptr_t;
	typedef SensitivitySequentialRiccatiEquations<STATE_DIM, INPUT_DIM> SensitivityRiccatiEquations_t;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::template LinearFunction_t<INPUT_DIM, Eigen::Dynamic> sensitivity_controller_t;
	typedef typename DIMENSIONS::template linearFunction_array_t<INPUT_DIM, Eigen::Dynamic> sensitivity_controller_array_t;
	typedef typename DIMENSIONS::template LinearFunction_t<Eigen::Dynamic> lagrange_t;
	typedef typename DIMENSIONS::controller_t controller_t;
	typedef typename DIMENSIONS::controller_array_t controller_array_t;
	typedef typename DIMENSIONS::Options Options_t;
	typedef typename DIMENSIONS::scalar_t 		scalar_t;
	typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
	typedef typename DIMENSIONS::eigen_scalar_t       eigen_scalar_t;
	typedef typename DIMENSIONS::eigen_scalar_array_t eigen_scalar_array_t;
	typedef typename DIMENSIONS::eigen_scalar_array2_t eigen_scalar_array2_t;
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
	typedef typename DIMENSIONS::constraint1_vector_t       constraint1_vector_t;
	typedef typename DIMENSIONS::constraint1_vector_array_t constraint1_vector_array_t;
	typedef typename DIMENSIONS::constraint1_vector_array2_t constraint1_vector_array2_t;
	typedef typename DIMENSIONS::constraint1_state_matrix_t       constraint1_state_matrix_t;
	typedef typename DIMENSIONS::constraint1_state_matrix_array_t constraint1_state_matrix_array_t;
	typedef typename DIMENSIONS::constraint1_control_matrix_t       constraint1_control_matrix_t;
	typedef typename DIMENSIONS::constraint1_control_matrix_array_t constraint1_control_matrix_array_t;
	typedef typename DIMENSIONS::control_constraint1_matrix_t       control_constraint1_matrix_t;
	typedef typename DIMENSIONS::control_constraint1_matrix_array_t control_constraint1_matrix_array_t;
	typedef typename DIMENSIONS::constraint2_vector_t       constraint2_vector_t;
	typedef typename DIMENSIONS::constraint2_vector_array_t constraint2_vector_array_t;
	typedef typename DIMENSIONS::constraint2_vector_array2_t constraint2_vector_array2_t;
	typedef typename DIMENSIONS::constraint2_state_matrix_t       constraint2_state_matrix_t;
	typedef typename DIMENSIONS::constraint2_state_matrix_array_t constraint2_state_matrix_array_t;


	typedef RolloutSensitivityEquations<STATE_DIM, INPUT_DIM> RolloutSensitivityEquations_t;
	typedef typename RolloutSensitivityEquations_t::nabla_state_matrix_t	   nabla_state_matrix_t;
	typedef typename RolloutSensitivityEquations_t::nabla_state_matrix_array_t nabla_state_matrix_array_t;
	typedef typename RolloutSensitivityEquations_t::nabla_input_matrix_t       nabla_input_matrix_t;
	typedef typename RolloutSensitivityEquations_t::nabla_input_matrix_array_t nabla_input_matrix_array_t;
	typedef typename RolloutSensitivityEquations_t::nabla_scalar_rowvector_t       nabla_scalar_rowvector_t;
	typedef typename RolloutSensitivityEquations_t::nabla_scalar_rowvector_array_t nabla_scalar_rowvector_array_t;
	typedef Eigen::Matrix<double,DIMENSIONS::MAX_CONSTRAINT1_DIM_,Eigen::Dynamic>  nabla_constraint1_matrix_t;
	typedef std::vector<nabla_constraint1_matrix_t> nabla_constraint1_matrix_array_t;

	typedef state_matrix_array_t nabla_Sm_t;
	typedef state_vector_array_t nabla_Sv_t;
	typedef eigen_scalar_array_t nabla_s_t;
	typedef std::vector<nabla_Sm_t> nabla_Sm_array_t;
	typedef std::vector<nabla_Sv_t> nabla_Sv_array_t;
    typedef std::vector<nabla_s_t>  nabla_s_array_t;

    GSLQP()
    : numSubsystems_(0)
    {}

	GSLQP(const Options_t& options  = Options_t())
    : options_(options),
      numSubsystems_(0)
    {}

	~GSLQP() {}

	void rollout(const scalar_t& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const controller_array_t& controllersStock,
			std::vector<scalar_array_t>& timeTrajectoriesStock,
			state_vector_array2_t& stateTrajectoriesStock,
			control_vector_array2_t& inputTrajectoriesStock,
			std::vector<std::vector<size_t> >& nc1TrajectoriesStock,
			constraint1_vector_array2_t& EvTrajectoryStock,
			std::vector<std::vector<size_t> >& nc2TrajectoriesStock,
			constraint2_vector_array2_t& HvTrajectoryStock,
			std::vector<size_t>& nc2FinalStock,
			constraint2_vector_array_t& HvFinalStock);


	void rollout(const scalar_t& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const controller_array_t& controllersStock,
			std::vector<scalar_array_t>& timeTrajectoriesStock,
			state_vector_array2_t& stateTrajectoriesStock,
			control_vector_array2_t& inputTrajectoriesStock);

	void calculateCostFunction(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const control_vector_array2_t& inputTrajectoriesStock,
			scalar_t& totalCost);

	void calculateMeritFunction(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const std::vector<std::vector<size_t> >& nc1TrajectoriesStock,
			const constraint1_vector_array2_t& EvTrajectoryStock,
			const std::vector<std::vector<Eigen::VectorXd> >&  lagrangeTrajectoriesStock,
			const scalar_t& totalCost,
			scalar_t& meritFuntionValue,
			scalar_t& constraintISE);

	double calculateConstraintISE(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const std::vector<std::vector<size_t>>& nc1TrajectoriesStock,
			const constraint1_vector_array2_t& EvTrajectoriesStock,
			scalar_t& constraintISE);

	void getRolloutSensitivity2SwitchingTime(std::vector<scalar_array_t>& sensitivityTimeTrajectoriesStock,
		std::vector<nabla_state_matrix_array_t>& sensitivityStateTrajectoriesStock,
		std::vector<nabla_input_matrix_array_t>& sensitivityInputTrajectoriesStock);

	void getController(controller_array_t& controllersStock);

	void getValueFuntion(const scalar_t& time, const state_vector_t& state, scalar_t& valueFuntion);

	void getCostFuntion(scalar_t& costFunction, scalar_t& constriantISE);

	void getValueFuntionDerivative(const state_vector_t& initState, Eigen::VectorXd& valueFuntionDerivative);

	void getCostFuntionDerivative(Eigen::VectorXd& costFuntionDerivative);

	void getNominalTrajectories(std::vector<scalar_array_t>& nominalTimeTrajectoriesStock,
			state_vector_array2_t& nominalStateTrajectoriesStock,
			control_vector_array2_t& nominalInputTrajectoriesStock);

	void run(const double& initTime, const state_vector_t& initState, const double& finalTime,
			const std::vector<size_t>& systemStockIndexes,
			const std::vector<scalar_t>& switchingTimes,
			const slqp_ptr_t& slqpPtr,
			const std::vector<scalar_array_t>& desiredTimeTrajectoriesStock = std::vector<scalar_array_t>(),
			const state_vector_array2_t& desiredStateTrajectoriesStock = state_vector_array2_t());

protected:

	void runLQBasedMethod();

	void runSweepingBVPMethod();

	void solveSensitivityRiccatiEquations(const scalar_t& learningRate);

	void transformLocalValueFuntionDerivative2Global();

	void rolloutSensitivity2SwitchingTime(const sensitivity_controller_array_t& sensitivityControllersStock,
			std::vector<scalar_array_t>& sensitivityTimeTrajectoryStock,
			std::vector<nabla_state_matrix_array_t>& nablaStateTrajectoryStock,
			std::vector<nabla_input_matrix_array_t>& nablaInputTrajectoryStock);

	void approximateNominalLQPSensitivity2SwitchingTime();

	void calculateSensitivityControllerFeedback(sensitivity_controller_array_t& sensitivityControllersStock);

	void calculateLQSensitivityControllerForward(sensitivity_controller_array_t& sensitivityControllersStock);

	void calculateBVPSensitivityControllerForward(const size_t& switchingTimeIndex, const state_vector_array2_t& SvTrajectoriesStock,
			sensitivity_controller_array_t& sensitivityControllersStock);

	void calculateStateTimeDerivative();

	void solveSensitivityBVP(const size_t& switchingTimeIndex,
			const std::vector<scalar_array_t>& timeTrajectoriesStock,
			state_matrix_array2_t& MmTrajectoriesStock,
			state_vector_array2_t& SvTrajectoriesStock);

	void calculateBVPCostFunctionDerivative(Eigen::VectorXd& costFunctionDerivative);

private:

	slqp_ptr_t slqpPtr_;

	Eigen::VectorXd 		nominalCostFuntionDerivative_;
	state_vector_array2_t 	nominalStateTimeDerivativeTrajectoriesStock_;
	sensitivity_controller_array_t nominalSensitivityControllersStock_;

	std::vector<scalar_array_t> sensitivityTimeTrajectoryStock_;
	std::vector<nabla_state_matrix_array_t> nablaStateTrajectoryStock_;
	std::vector<nabla_input_matrix_array_t> nablaInputTrajectoryStock_;
	//
	std::vector<nabla_scalar_rowvector_array_t> nablaqTrajectoryStock_;
	std::vector<nabla_state_matrix_array_t> nablaQvTrajectoryStock_;
	std::vector<nabla_input_matrix_array_t> nablaRvTrajectoryStock_;
	std::vector<nabla_constraint1_matrix_array_t> nablaEvTrajectoryStock_;
	nabla_scalar_rowvector_array_t nablaqFinalStock_;
	nabla_state_matrix_array_t nablaQvFinalStock_;


	std::vector<nabla_s_array_t>  nablasTrajectoryStock_;
	std::vector<nabla_Sv_array_t> nablaSvTrajectoryStock_;
	std::vector<nabla_Sm_array_t> nablaSmTrajectoryStock_;

	scalar_array_t switchingTimes_;
	std::vector<size_t> systemStockIndexes_;

	state_vector_t 	initState_;
	scalar_t 		initTime_;
	scalar_t 		finalTime_;
	size_t 			numSubsystems_;
	Options_t 		options_;
};

} // namespace ocs2

#include "implementation/GSLQP.h"

#endif /* GSLQP_H_ */
