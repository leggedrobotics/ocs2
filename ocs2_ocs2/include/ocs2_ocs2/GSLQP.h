/**
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

/**
 * GSLQP Class
 * @tparam STATE_DIM
 * @tparam INPUT_DIM
 */
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

    /**
     * Forward integrate the system dynamics with given controller:
     * 		inputs:
     * 			+ initState: initial state at time switchingTimes_[0]
     * 			+ controllersStock: controller for each subsystem
     * 		outputs:
     * 			+ timeTrajectoriesStock:  rollout simulated time steps
     * 			+ stateTrajectoriesStock: rollout states
     * 			+ inputTrajectoriesStock: rollout control inputs
     * 			+ (optional) nc1TrajectoriesStock: number of active constraints at each time step
     * 			+ (optional) EvTrajectoryStock: value of the constraint (if the rollout is constrained the value is
     * 											always zero otherwise it is nonzero)
     * @param [in] initTime
     * @param [in] initState
     * @param [in] finalTime
     * @param [in] controllersStock
     * @param [out] timeTrajectoriesStock
     * @param [out] stateTrajectoriesStock
     * @param [out] inputTrajectoriesStock
     * @param [out] nc1TrajectoriesStock
     * @param [out] EvTrajectoryStock
     * @param [out] nc2TrajectoriesStock
     * @param [out] HvTrajectoryStock
     * @param [out] nc2FinalStock
     * @param [out] HvFinalStock
     */
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

    /**
     * Rollout function
     * @param [in] initTime
     * @param [in] initState
     * @param [in] finalTime
     * @param [in] controllersStock
     * @param [out] timeTrajectoriesStock
     * @param [out] stateTrajectoriesStock
     * @param [out] inputTrajectoriesStock
     */
	void rollout(const scalar_t& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const controller_array_t& controllersStock,
			std::vector<scalar_array_t>& timeTrajectoriesStock,
			state_vector_array2_t& stateTrajectoriesStock,
			control_vector_array2_t& inputTrajectoriesStock);

    /**
     * compute the cost for a given rollout
     * 		inputs:
     * 			+ timeTrajectoriesStock:  rollout simulated time steps
     * 			+ stateTrajectoriesStock: rollout states
     * 			+ inputTrajectoriesStock: rollout control inputs
     *
     * 		outputs:
     * 			+ totalCost: the total cost of the trajectory
     * @param [in] timeTrajectoriesStock
     * @param [in] stateTrajectoriesStock
     * @param [in] inputTrajectoriesStock
     * @param [out] totalCost
     */
	void calculateCostFunction(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const control_vector_array2_t& inputTrajectoriesStock,
			scalar_t& totalCost);

    /**
     * compute the merit function for given rollout
     * 		inputs:
     * 			+ timeTrajectoriesStock: simulation time trajectory
     * 			+ nc1TrajectoriesStock: rollout's number of active constraints in each time step
     * 			+ EvTrajectoryStock: rollout's constraints value
     * 			+ lagrangeTrajectoriesStock: constraint Lagrange multiplier for the given rollout
     * 			+ totalCost: the total cost of the trajectory
     *
     * 		outputs:
     * 			+ meritFuntionValue: the merit function value
     * 			+ constraintISE: integral of Square Error (ISE)
     * @param [in] timeTrajectoriesStock
     * @param [in] nc1TrajectoriesStock
     * @param [in] EvTrajectoryStock
     * @param [in] lagrangeTrajectoriesStock
     * @param [in] totalCost
     * @param [out] meritFuntionValue
     * @param [out] constraintISE
     */
	void calculateMeritFunction(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const std::vector<std::vector<size_t> >& nc1TrajectoriesStock,
			const constraint1_vector_array2_t& EvTrajectoryStock,
			const std::vector<std::vector<Eigen::VectorXd> >&  lagrangeTrajectoriesStock,
			const scalar_t& totalCost,
			scalar_t& meritFuntionValue,
			scalar_t& constraintISE);

    /**
     * Constraint's Integral of Squared Error (ISE)
     * 		Inputs:
     * 			+ timeTrajectoriesStock: simulation time trajectory
     * 			+ nc1TrajectoriesStock: rollout's number of active constraints in each time step
     * 			+ EvTrajectoriesStock: rollout's constraints value
     * 		output:
     * 			+ constraintISE: integral of Square Error (ISE)
     * 		Return:
     * 			+ maximum constraint norm
     * @param [in] timeTrajectoriesStock
     * @param [in] nc1TrajectoriesStock
     * @param [in] EvTrajectoriesStock
     * @param [out] constraintISE
     * @return double
     */
	double calculateConstraintISE(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const std::vector<std::vector<size_t>>& nc1TrajectoriesStock,
			const constraint1_vector_array2_t& EvTrajectoriesStock,
			scalar_t& constraintISE);

    /**
     * get the calculated rollout's sensitivity to switchingTimes
     * 		outputs:
     * 			+ sensitivityTimeTrajectoriesStock: time stamps of the sensitivity values
     * 			+ sensitivityStateTrajectoriesStock: state trajectory sensitivity to the switching times
     * 			+ sensitivityInputTrajectoriesStock: control input trajectory sensitivity to the switching times
     * @param [out] sensitivityTimeTrajectoriesStock
     * @param [out] sensitivityStateTrajectoriesStock
     * @param [out] sensitivityInputTrajectoriesStock
     */
	void getRolloutSensitivity2SwitchingTime(std::vector<scalar_array_t>& sensitivityTimeTrajectoriesStock,
		std::vector<nabla_state_matrix_array_t>& sensitivityStateTrajectoriesStock,
		std::vector<nabla_input_matrix_array_t>& sensitivityInputTrajectoriesStock);

    /**
     * get the calculated optimal controller structure
     * @param [out] controllersStock
     */
	void getController(controller_array_t& controllersStock);

    /**
     * calculate the value function for the given time and state vector
     * 		inputs
     * 			+ time: inquiry time
     * 			+ state: inquiry state
     *
     * 		output:
     * 			+ valueFuntion: value function at the inquiry time and state
     * @param [in] time
     * @param [in] state
     * @param [out] valueFuntion
     */
	void getValueFuntion(const scalar_t& time, const state_vector_t& state, scalar_t& valueFuntion);

    /**
     * calculate the cost function at the initial time
     * 		inputs
     * 			+ initState: initial state
     *
     * 		output:
     * 			+ cost function value
     * 			+ cost function value plus the constraint ISE multiplied by pho
     * @param [in] costFunction
     * @param [out] constriantISE
     */
	void getCostFuntion(scalar_t& costFunction, scalar_t& constriantISE);

    /**
     * calculate the value function's derivatives w.r.t. switchingTimes at the initial time
     * 		inputs
     * 			+ initState: initial state
     *
     * 		output:
     * 			+ valueFuntionDerivative: cost function' derivatives w.r.t. switchingTimes for given initial state vector
     * @param [in] initState
     * @param [out] valueFuntionDerivative
     */
	void getValueFuntionDerivative(const state_vector_t& initState, Eigen::VectorXd& valueFuntionDerivative);

    /**
     * calculate the cost function's derivatives w.r.t. switchingTimes
     *
     * 		output:
     * 			+ costFunctionDerivative: cost function' derivatives w.r.t. switchingTimes for given initial state vector
     * @param [out] costFuntionDerivative
     */
	void getCostFuntionDerivative(Eigen::VectorXd& costFuntionDerivative);

    /**
     * get the optimal state and input trajectories
     * 		output
     * 			+ nominalTimeTrajectoriesStock_: optimal time trajectory
     * 			+ nominalStateTrajectoriesStock_: optimal state trajectory
     * 			+ nominalInputTrajectoriesStock_: optimal control input trajectory
     * @param [out] nominalTimeTrajectoriesStock
     * @param [out] nominalStateTrajectoriesStock
     * @param [out] nominalInputTrajectoriesStock
     */
	void getNominalTrajectories(std::vector<scalar_array_t>& nominalTimeTrajectoriesStock,
			state_vector_array2_t& nominalStateTrajectoriesStock,
			control_vector_array2_t& nominalInputTrajectoriesStock);

    /**
     * run the SLQ algorithm for a given state and switching times based on the BVP method
     * @param [in] initTime
     * @param [in] initState
     * @param [in] finalTime
     * @param [in] systemStockIndexes
     * @param [in] switchingTimes
     * @param [in] slqpPtr
     * @param [in] desiredTimeTrajectoriesStock
     * @param [in] desiredStateTrajectoriesStock
     */
	void run(const double& initTime, const state_vector_t& initState, const double& finalTime,
			const std::vector<size_t>& systemStockIndexes,
			const std::vector<scalar_t>& switchingTimes,
			const slqp_ptr_t& slqpPtr,
			const std::vector<scalar_array_t>& desiredTimeTrajectoriesStock = std::vector<scalar_array_t>(),
			const state_vector_array2_t& desiredStateTrajectoriesStock = state_vector_array2_t());

protected:

    /**
     * run the LQ-based algorithm to compute the cost function derivatives wr.t. switchingTimes
     */
	void runLQBasedMethod();

    /**
     * run the Sweeping-BVP algorithm to compute the cost function derivatives wr.t. switchingTimes
     */
	void runSweepingBVPMethod();

    /**
     * solve the SLQ Riccati differential equations plus its derivatives differential equations :
     * 		input:
     * 			+ learningRate: the feeadforward learningRate
     *
     * 		uses:
     * 			+ linearized dynamics
     * 			+ quadratized cost
     * 			+ nominal system sensitivity analysis
     * 			+ SsTimeTrajectoryStock_: time stamp
     * 			V(t,y) = y^T*Sm*y + y^T*(Sv) + s
     * 			+ SmTrajectoryStock_: Sm matrix
     * 			+ SvTrajectoryStock_: Sv vector
     *
     * 		modifies:

     * 			dV(t,y) = y^T*dSm*y + y^T*(dSv) + ds
     * 			+ nablaSmTrajectoryStock_: dSm
     * 			+ nablaSvTrajectoryStock_: dSv
     * 			+ nablasTrajectoryStock_: ds
     * @param [out] learningRate
     */
	void solveSensitivityRiccatiEquations(const scalar_t& learningRate);

    /**
     * transform the local value function derivatives to the global one.
     * 		it manipulates the following member variables:
     * 			+ nablasTrajectoryStock_
     * 			+ nablaSvTrajectoryStock_
     */
	void transformLocalValueFuntionDerivative2Global();

    /**
     * calculates the sensitivity of the rollout and LQ model to the switchingTimes
     * 		inputs:
     * 			+ sensitivityControllersStock
     * 		outputs:
     * 			+ sensitivityTimeTrajectoryStock: time stamp
     * 			+ nablaStateTrajectoryStock: dy
     * 			+ nablaInputTrajectoryStock: du
     * @param [in] sensitivityControllersStock
     * @param [out] sensitivityTimeTrajectoryStock
     * @param [out] nablaStateTrajectoryStock
     * @param [out] nablaInputTrajectoryStock
     */
	void rolloutSensitivity2SwitchingTime(const sensitivity_controller_array_t& sensitivityControllersStock,
			std::vector<scalar_array_t>& sensitivityTimeTrajectoryStock,
			std::vector<nabla_state_matrix_array_t>& nablaStateTrajectoryStock,
			std::vector<nabla_input_matrix_array_t>& nablaInputTrajectoryStock);

    /**
     * approximate nominal LQ problem sensitivity to switching times
     * 		modifies:
     * 			+ nablaqTrajectoryStock_: dq
     * 			+ nablaQvTrajectoryStock_: dQv
     * 			+ nablaRvTrajectoryStock_: dRv
     * 			+ nablaqFinalStock_: dq_f
     * 			+ nablaQvFinalStock_: dQv_f
     */
	void approximateNominalLQPSensitivity2SwitchingTime();

    /**
     * calculates sensitivity controller feedback part (constrained feedback):
     * 		This method uses the following variables:
     * 			+ constrained, linearized model
     * 			+ constrained, quadratized cost
     *
     * 		output:
     * 			+ sensitivityControllersStock: the sensitivity controller
     * @param [out] sensitivityControllersStock
     */
	void calculateSensitivityControllerFeedback(sensitivity_controller_array_t& sensitivityControllersStock);

    /**
     * calculate the sensitivity of the control input increment to switchingTimes based on the LQ method
     * 		input & output:
     * 			+ sensitivityControllersStock
     * @param [in] sensitivityControllersStock
     */
	void calculateLQSensitivityControllerForward(sensitivity_controller_array_t& sensitivityControllersStock);

    /**
     * calculate the sensitivity of the control input increment to switchingTimes based on the BVP method
     * 		inputs
     * 			+ switchingTimeIndex: the index of the switching time which the cost derivative will be calculated
     * 			+ SvTrajectoriesStock: sweeping method S vector
     *
     * 		output:
     * 			+ sensitivityControllersStock
     * @param [in] switchingTimeIndex
     * @param [in] SvTrajectoriesStock
     * @param [out] sensitivityControllersStock
     */
	void calculateBVPSensitivityControllerForward(const size_t& switchingTimeIndex, const state_vector_array2_t& SvTrajectoriesStock,
			sensitivity_controller_array_t& sensitivityControllersStock);

    /**
     * calculate the nominal state time derivative
     */
	void calculateStateTimeDerivative();

    /**
     * solve sensitivity BVP (the boundary value problem of the sensitivity equations for a given switching time)
     * 		inputs
     * 			+ switchingTimeIndex: the index of the switching time which the cost derivative will be calculated
     * 			+ timeTrajectoriesStock: time stamp
     *
     * 		outputs
     * 			+ MmTrajectoriesStock: sweeping method M matrix
     * 			+ SvTrajectoriesStock: sweeping method S vector
     *
     * 		uses
     * 			+ linearized dynamics
     * 			+ quadratized cost
     * @param [in] switchingTimeIndex
     * @param [in] timeTrajectoriesStock
     * @param [out] MmTrajectoriesStock
     * @param [out] SvTrajectoriesStock
     */
	void solveSensitivityBVP(const size_t& switchingTimeIndex,
			const std::vector<scalar_array_t>& timeTrajectoriesStock,
			state_matrix_array2_t& MmTrajectoriesStock,
			state_vector_array2_t& SvTrajectoriesStock);

    /**
     * calculates cost function derivative based on BVP solution
     * @param [out] costFunctionDerivative
     */
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
