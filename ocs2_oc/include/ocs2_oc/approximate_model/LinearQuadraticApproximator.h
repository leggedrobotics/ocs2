/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#ifndef LINEAR_QUADRATIC_APPROXIMATOR_OCS2_H_
#define LINEAR_QUADRATIC_APPROXIMATOR_OCS2_H_

#include <memory>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/integration/Integrator.h>
#include <ocs2_core/integration/SystemEventHandler.h>

namespace ocs2 {

/**
 * This class is an interface class for constructing LQ approximation of the continous time optimal control problem.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules>
class LinearQuadraticApproximator
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static_assert(std::is_base_of<LogicRulesBase, LOGIC_RULES_T>::value,
			"LOGIC_RULES_T must inherit from LogicRulesBase");

	typedef Dimensions<STATE_DIM, INPUT_DIM>                          DIMENSIONS;
	typedef DerivativesBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>      derivatives_base_t;
	typedef ConstraintBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>       constraint_base_t;
	typedef CostFunctionBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>     cost_function_base_t;

	typedef typename DIMENSIONS::scalar_t                      scalar_t;
	typedef typename DIMENSIONS::scalar_array_t                scalar_array_t;
	typedef typename DIMENSIONS::state_vector_t                state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t          state_vector_array_t;
	typedef typename DIMENSIONS::input_vector_t                input_vector_t;
	typedef typename DIMENSIONS::input_vector_array_t          input_vector_array_t;
	typedef typename DIMENSIONS::eigen_scalar_t                eigen_scalar_t;
	typedef typename DIMENSIONS::input_state_matrix_t          input_state_matrix_t;
	typedef typename DIMENSIONS::input_state_matrix_array_t    input_state_matrix_array_t;
	typedef typename DIMENSIONS::state_matrix_t                state_matrix_t;
	typedef typename DIMENSIONS::state_matrix_array_t          state_matrix_array_t;
	typedef typename DIMENSIONS::input_matrix_t                input_matrix_t;
	typedef typename DIMENSIONS::input_matrix_array_t          input_matrix_array_t;
	typedef typename DIMENSIONS::state_input_matrix_t          state_input_matrix_t;
	typedef typename DIMENSIONS::constraint1_vector_t          constraint1_vector_t;
	typedef typename DIMENSIONS::constraint1_state_matrix_t    constraint1_state_matrix_t;
	typedef typename DIMENSIONS::constraint1_input_matrix_t    constraint1_input_matrix_t;
	typedef typename DIMENSIONS::input_constraint1_matrix_t  input_constraint1_matrix_t;
	typedef typename DIMENSIONS::constraint2_vector_t          constraint2_vector_t;
	typedef typename DIMENSIONS::constraint2_state_matrix_t    constraint2_state_matrix_t;

	/**
	 * Constructor
	 *
	 * @param [in] systemDerivatives: The system dynamics derivatives for subsystems of the system.
	 * @param [in] systemConstraints: The system constraint function and its derivatives for subsystems.
	 * @param [in] costFunction: The cost function (intermediate and terminal costs) and its derivatives for subsystems.
	 */
	LinearQuadraticApproximator(
			const derivatives_base_t& systemDerivatives,
			const constraint_base_t& systemConstraints,
			const cost_function_base_t& costFunction,
			const char algorithmName[] = nullptr,
			bool checkNumericalCharacteristics = true)
	: systemDerivativesPtr_(systemDerivatives.clone())
	, systemConstraintsPtr_(systemConstraints.clone())
	, costFunctionPtr_(costFunction.clone())
	, algorithmName_(algorithmName)
	, checkNumericalCharacteristics_(checkNumericalCharacteristics)
	{}


	/**
	 * Default destructor.
	 */
	~LinearQuadraticApproximator() = default;

	/**
	 * Whether or not to check the numerical characteristics of the model.
	 *
	 * @param [in] checkNumericalCharacteristics: True if the numerical characteristics of the model should be checked.
	 */
	void checkNumericalCharacteristics(bool checkNumericalCharacteristics) {

		checkNumericalCharacteristics_ = checkNumericalCharacteristics;
	}

	/**
	 * Initializes the system derivatives, constarints, and cost.
	 *
	 * @param [in] logicRulesMachine: A class which contains and parse the logic rules e.g
	 * method findActiveSubsystemHandle returns a Lambda expression which can be used to
	 * find the ID of the current active subsystem.
	 * @param [in] partitionIndex: index of the time partition.
	 * @param [in] algorithmName: The algorithm that class this class (default not defined).
	 */
	virtual void initializeModel(
			LogicRulesMachine<LOGIC_RULES_T>& logicRulesMachine,
			const size_t& partitionIndex,
			const char* algorithmName=NULL)
	{
		// initialize subsystem i dynamics derivatives
		systemDerivativesPtr_->initializeModel(logicRulesMachine, partitionIndex, "algorithmName");
		// initialize subsystem i constraint
		systemConstraintsPtr_->initializeModel(logicRulesMachine, partitionIndex, "algorithmName");
		// initialize subsystem i cost
		costFunctionPtr_->initializeModel(logicRulesMachine, partitionIndex, "algorithmName");
	}

	/**
	 * Returns the system derivatives
	 */
	derivatives_base_t& systemDerivatives() {

		return *systemDerivativesPtr_;
	}

	/**
	 * Returns the constraints.
	 */
	constraint_base_t& systemConstraints() {

		return *systemConstraintsPtr_;
	}

	/**
	 * Returns the intermediate cost.
	 */
	cost_function_base_t& costFunction() {

		return *costFunctionPtr_;
	}

	/**
	 * Calculates an LQ approximate of the unconstrained optimal control problem at a given time, state, and input.
	 *
	 * @param [in] time: The current time.
	 * @param [in] state: The current state.
	 * @param [in] input: The current input .
	 */
	void approximateUnconstrainedLQProblem(
			const scalar_t& time,
			const state_vector_t& state,
			const input_vector_t& input) {

		approximateDynamics(time, state, input);
		approximateConstraints(time, state, input);
		approximateIntermediateCost(time, state, input);
	}

	/**
	 * Calculates an LQ approximate of the event times process.
	 *
	 * @param [in] time: The current time.
	 * @param [in] state: The current state.
	 * @param [in] input: The current input .
	 */
	void approximateUnconstrainedLQProblemAtEventTime(
			const scalar_t& time,
			const state_vector_t& state,
			const input_vector_t& input) {

		systemConstraintsPtr_->setCurrentStateAndControl(time, state, input);
		systemConstraintsPtr_->setCurrentStateAndControl(time, state, input);

		// Final state-only equality constraint
		ncFinalEqStateOnly_ = systemConstraintsPtr_->numStateOnlyFinalConstraint(time);

		if (ncFinalEqStateOnly_ > INPUT_DIM)
			throw std::runtime_error("Number of active final type-2 constraints should be "
					"less-equal to the number of input dimension.");

		// if final constraint type 2 is active
		if (ncFinalEqStateOnly_ > 0) {
			systemConstraintsPtr_->getFinalConstraint2(HvFinal_);
			systemConstraintsPtr_->getFinalConstraint2DerivativesState(FmFinal_);
		}

		// Final cost
		costFunctionPtr_->getTerminalCost(qFinal_(0));
		costFunctionPtr_->getTerminalCostDerivativeState(QvFinal_);
		costFunctionPtr_->getTerminalCostSecondDerivativeState(QmFinal_);
	}

	/**
	 * Calculates linearize system dynamics.
	 *
	 * @param [in] time: The current time.
	 * @param [in] state: The current state.
	 * @param [in] input: The current input .
	 */
	void approximateDynamics(
			const scalar_t& time,
			const state_vector_t& state,
			const input_vector_t& input) {


		// set data
		systemDerivativesPtr_->setCurrentStateAndControl(time, state, input);

		// get results
		systemDerivativesPtr_->getFlowMapDerivativeState(Am_);
		systemDerivativesPtr_->getFlowMapDerivativeInput(Bm_);

		// checking the numerical stability
		if (checkNumericalCharacteristics_==true){
			try {
				if (!Am_.allFinite())
					throw std::runtime_error("Flow map state derivativeState is not finite.");
				if (!Bm_.allFinite())
					throw std::runtime_error("Flow map input derivativeState is not finite.");

			} catch(const std::exception& error)  {
				std::cerr << "what(): " << error.what() << " at time " << time << " [sec]." << std::endl;
				std::cerr << "Am: \n" << Am_ << std::endl;
				std::cerr << "Bm: \n" << Bm_ << std::endl;
				exit(0);
			}
		}

	}

	/**
	 * Calculates the constraints and its linearized approximation.
	 *
	 * @param [in] time: The current time.
	 * @param [in] state: The current state.
	 * @param [in] input: The current input .
	 */
	void approximateConstraints(
			const scalar_t& time,
			const state_vector_t& state,
			const input_vector_t& input) {

		// set data
		systemConstraintsPtr_->setCurrentStateAndControl(time, state, input);

		// constraint type 1
		ncEqStateInput_ = systemConstraintsPtr_->numStateInputConstraint(time);
		if (ncEqStateInput_ > INPUT_DIM)
			throw std::runtime_error("Number of active type-1 constraints should be less-equal to the number of input dimension.");
		// if constraint type 1 is active
		if (ncEqStateInput_ > 0) {
			systemConstraintsPtr_->getConstraint1(Ev_);
			systemConstraintsPtr_->getConstraint1DerivativesState(Cm_);
			systemConstraintsPtr_->getConstraint1DerivativesControl(Dm_);
		}

		// constraint type 2
		ncEqStateOnly_ = systemConstraintsPtr_->numStateOnlyConstraint(time);
		if (ncEqStateOnly_ > INPUT_DIM)
			throw std::runtime_error("Number of active type-2 constraints should be less-equal to the number of input dimension.");
		// if constraint type 2 is active
		if (ncEqStateOnly_ > 0) {
			systemConstraintsPtr_->getConstraint2(Hv_);
			systemConstraintsPtr_->getConstraint2DerivativesState(Fm_);
		}

		// Inequality constraint
		ncIneq_ = systemConstraintsPtr_->numInequalityConstraint(time);
		if (ncIneq_ > 0){
			systemConstraintsPtr_->getInequalityConstraint(h_);
			systemConstraintsPtr_->getInequalityConstraintDerivativesState(dhdx_);
			systemConstraintsPtr_->getInequalityConstraintDerivativesInput(dhdu_);
			systemConstraintsPtr_->getInequalityConstraintSecondDerivativesState(ddhdxdx_);
			systemConstraintsPtr_->getInequalityConstraintSecondDerivativesInput(ddhdudu_);
			systemConstraintsPtr_->getInequalityConstraintDerivativesInputState(ddhdudx_);
		}

		if (checkNumericalCharacteristics_==true){
			try {
				if (ncEqStateInput_ > 0) {
					if (!Ev_.head(ncEqStateInput_).allFinite())
						throw std::runtime_error("Input-state constraint is not finite.");
					if (!Cm_.topRows(ncEqStateInput_).allFinite())
						throw std::runtime_error("Input-state constraint derivative w.r.t. state is not finite.");
					if (!Dm_.topRows(ncEqStateInput_).allFinite())
						throw std::runtime_error("Input-state constraint derivative w.r.t. input is not finite.");
					size_t DmRank = Dm_.topRows(ncEqStateInput_).colPivHouseholderQr().rank();
					if (DmRank != ncEqStateInput_)
						throw std::runtime_error("Input-state constraint derivative w.r.t. input is not full-row rank. It's rank "
								"is " + std::to_string(DmRank) + " while the expected rank is " + std::to_string(ncEqStateInput_) + ".");
				}
				if (ncEqStateOnly_ > 0) {
					if (!Hv_.head(ncEqStateOnly_).allFinite())
						throw std::runtime_error("State-only constraint is not finite.");
					if (!Fm_.topRows(ncEqStateOnly_).allFinite())
						throw std::runtime_error("State-only constraint derivative w.r.t. state is not finite.");
				}

			} catch(const std::exception& error)  {
				std::cerr << "what(): " << error.what() << " at time " << time << " [sec]." << std::endl;
				std::cerr << "Ev: " << Ev_.head(ncEqStateInput_).transpose() << std::endl;
				std::cerr << "Cm: \n" << Cm_.topRows(ncEqStateInput_) << std::endl;
				std::cerr << "Dm: \n" << Dm_.topRows(ncEqStateInput_) << std::endl;
				std::cerr << "Hv: " << Hv_.head(ncEqStateOnly_).transpose() << std::endl;
				std::cerr << "Fm: \n" << Fm_.topRows(ncEqStateOnly_) << std::endl;
				exit(0);
			}
		}

	}

	/**
	 * Calculates the intermediate cost function and its quadratic approximation.
	 *
	 * @param [in] time: The current time.
	 * @param [in] state: The current state.
	 * @param [in] input: The current input .
	 */
	void approximateIntermediateCost(
			const scalar_t& time,
			const state_vector_t& state,
			const input_vector_t& input) {


		// set data
		costFunctionPtr_->setCurrentStateAndControl(time, state, input);

		// get results
		costFunctionPtr_->getIntermediateCost(q_(0));
		costFunctionPtr_->getIntermediateCostDerivativeState(Qv_);
		costFunctionPtr_->getIntermediateCostSecondDerivativeState(Qm_);
		costFunctionPtr_->getIntermediateCostDerivativeInput(Rv_);
		costFunctionPtr_->getIntermediateCostSecondDerivativeInput(Rm_);
		costFunctionPtr_->getIntermediateCostDerivativeInputState(Pm_);

		// checking the numerical stability
		if (checkNumericalCharacteristics_==true){
			try {
				if (!q_.allFinite())
					throw std::runtime_error("Intermediate cost is is not finite.");
				if (!Qv_.allFinite())
					throw std::runtime_error("Intermediate cost first derivative w.r.t. state is is not finite.");
				if (!Qm_.allFinite())
					throw std::runtime_error("Intermediate cost second derivative w.r.t. state is is not finite.");
				if (!Qm_.isApprox(Qm_.transpose()))
					throw std::runtime_error("Intermediate cost second derivative w.r.t. state is is not self-adjoint.");
				if (Qm_.eigenvalues().real().minCoeff() < -Eigen::NumTraits<scalar_t>::epsilon())
					throw std::runtime_error("Q matrix is not positive semi-definite. It's smallest eigenvalue is " +
							std::to_string(Qm_.eigenvalues().real().minCoeff()) + ".");
				if (!Rv_.allFinite())
					throw std::runtime_error("Intermediate cost first derivative w.r.t. input is is not finite.");
				if (!Rm_.allFinite())
					throw std::runtime_error("Intermediate cost second derivative w.r.t. input is is not finite.");
				if (!Rm_.isApprox(Rm_.transpose()))
					throw std::runtime_error("Intermediate cost second derivative w.r.t. input is is not self-adjoint.");
				if (!Pm_.allFinite())
					throw std::runtime_error("Intermediate cost second derivative w.r.t. input-state is is not finite.");
				if (Rm_.ldlt().rcond() < Eigen::NumTraits<scalar_t>::epsilon())
					throw std::runtime_error("R matrix is not invertible. It's reciprocal condition number is " +
							std::to_string(Rm_.ldlt().rcond()) + ".");
				if (Rm_.eigenvalues().real().minCoeff() < Eigen::NumTraits<scalar_t>::epsilon())
					throw std::runtime_error("R matrix is not positive definite. It's smallest eigenvalue is " +
							std::to_string(Rm_.eigenvalues().real().minCoeff()) + ".");
			} catch(const std::exception& error)  {
				std::cerr << "what(): " << error.what() << " at time " << time << " [sec]." << std::endl;
				std::cerr << "x: " << state.transpose() << std::endl;
				std::cerr << "u: " << input.transpose() << std::endl;
				std::cerr << "q: " << q_ << std::endl;
				std::cerr << "Qv: " << Qv_.transpose() << std::endl;
				std::cerr << "Qm: \n" << Qm_ << std::endl;
				std::cerr << "Rv: " << Rv_.transpose() << std::endl;
				std::cerr << "Rm: \n" << Rm_ << std::endl;
				std::cerr << "Pm: \n" << Pm_ << std::endl;
				exit(0);
			}
		}

		// Pre-compute R inverse after costs are adapted
		RmInverse_ = Rm_.ldlt().solve(input_matrix_t::Identity());
	}

public:
	state_matrix_t       Am_;
	state_input_matrix_t Bm_;

	size_t                     ncEqStateInput_; // Number of the state-input equality constraints
	constraint1_vector_t       Ev_;
	constraint1_state_matrix_t Cm_;
	constraint1_input_matrix_t Dm_;

	size_t                     ncEqStateOnly_;  // Number of the state-only equality constraints
	constraint2_vector_t       Hv_;
	constraint2_state_matrix_t Fm_;

	size_t                     ncIneq_;         // Number of inequality constraints
	scalar_array_t       	   h_;
	state_vector_array_t       dhdx_;
	state_matrix_array_t       ddhdxdx_;
	input_vector_array_t       dhdu_;
	input_matrix_array_t       ddhdudu_;
	input_state_matrix_array_t ddhdudx_;

	eigen_scalar_t       q_;
	state_vector_t       Qv_;
	state_matrix_t       Qm_;
	input_vector_t       Rv_;
	input_matrix_t       Rm_;
	input_state_matrix_t Pm_;
	input_matrix_t       RmInverse_;

	size_t                     ncFinalEqStateOnly_;
	constraint2_vector_t       HvFinal_;
	constraint2_state_matrix_t FmFinal_;

	eigen_scalar_t qFinal_;
	state_vector_t QvFinal_;
	state_matrix_t QmFinal_;


private:

	std::unique_ptr<derivatives_base_t> systemDerivativesPtr_;
	std::unique_ptr<constraint_base_t> systemConstraintsPtr_;
	std::unique_ptr<cost_function_base_t> costFunctionPtr_;

	const char* algorithmName_;
	bool checkNumericalCharacteristics_;
};

} // namespace ocs2

#endif /* LINEAR_QUADRATIC_APPROXIMATOR_OCS2_H_ */
