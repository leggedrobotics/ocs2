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
	typedef typename DIMENSIONS::control_constraint1_matrix_t  control_constraint1_matrix_t;
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
			const char* algorithmName = NULL)
	: systemDerivativesPtr_(systemDerivatives.clone())
	, systemConstraintsPtr_(systemConstraints.clone())
	, costFunctionPtr_(costFunction.clone())
	, algorithmName_(algorithmName)
	{}


	/**
	 * Default destructor.
	 */
	~LinearQuadraticApproximator() = default;

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
		if (checkNumericalStability_==true){
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

		if (checkNumericalStability_==true){
			try {
				if (ncEqStateInput_ > 0 && !Ev_.head(ncEqStateInput_).allFinite())
					throw std::runtime_error("Input-state constraint is not finite.");
				if (ncEqStateInput_ > 0 && !Cm_.topRows(ncEqStateInput_).allFinite())
					throw std::runtime_error("Input-state constraint derivative w.r.t. state is not finite.");
				if (ncEqStateInput_ > 0 && !Dm_.topRows(ncEqStateInput_).allFinite())
					throw std::runtime_error("Input-state constraint derivative w.r.t. input is not finite.");
				if (ncEqStateOnly_ > 0 && !Hv_.head(ncEqStateOnly_).allFinite())
					throw std::runtime_error("State-only constraint is not finite.");
				if (ncEqStateOnly_ > 0 && !Fm_.topRows(ncEqStateOnly_).allFinite())
					throw std::runtime_error("State-only constraint derivative w.r.t. state is not finite.");
				size_t DmRank = Dm_.topRows(ncEqStateInput_).colPivHouseholderQr().rank();
				if (DmRank != ncEqStateInput_)
					throw std::runtime_error("Input-state constraint derivative w.r.t. input is not full-row rank. It's rank "
							"is " + std::to_string(DmRank) + " while the expected rank is " + std::to_string(ncEqStateInput_) + ".");

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
		if (checkNumericalStability_==true){
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

	void approximateConstrainedLQWorker(
				size_t workerIndex,
				const size_t& i,
				const size_t& k,
				const scalar_t& stateConstraintPenalty) {

		// constraint type 2 coefficients
		if (ncEqStateOnly_ > 0) {
			q_  += 0.5 * stateConstraintPenalty *
					Hv_.head(ncEqStateOnly_).transpose() * Hv_.head(ncEqStateOnly_);
			Qv_ += stateConstraintPenalty *
					Fm_.topRows(ncEqStateOnly_).transpose() * Hv_.head(ncEqStateOnly_);
			Qm_ += stateConstraintPenalty *
					Fm_.topRows(ncEqStateOnly_).transpose() * Fm_.topRows(ncEqStateOnly_);
		}

		// Inequality constraints
		if (ncIneq_ > 0) {
			scalar_t p;
			state_vector_t dpdx;
			input_vector_t dpdu;
			state_matrix_t ddpdxdx;
			input_matrix_t ddpdudu;
			input_state_matrix_t ddpdudx;
			penaltyPtrStock_[workerIndex]->getPenaltyCost(h_, p);
			penaltyPtrStock_[workerIndex]->getPenaltyCostDerivativeState(h_, dhdx_,
															   dpdx);
			penaltyPtrStock_[workerIndex]->getPenaltyCostDerivativeInput(h_, dhdu_,
															   dpdu);
			penaltyPtrStock_[workerIndex]->getPenaltyCostSecondDerivativeState(h_,
																	 dhdx_,
																	 ddhdxdx_, ddpdxdx);
			penaltyPtrStock_[workerIndex]->getPenaltyCostSecondDerivativeInput(h_,
																	 dhdu_,
																	 ddhdudu_, ddpdudu);
			penaltyPtrStock_[workerIndex]->getPenaltyCostDerivativeInputState(h_,
																	dhdx_, dhdu_,
																	ddhdudx_, ddpdudx);
			q_[0] += p; // q is a 1x1 matrix, so access it with [0]
			Qv_ += dpdx;
			Qm_ += ddpdxdx;
			Rv_ += dpdu;
			Rm_ += ddpdudu;
			Pm_ += ddpdudx;

			// checking the numerical stability again
			if (checkNumericalStability_==true){
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
					std::cerr << "After adding inequality constraint penalty" << std::endl;
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
		}

		// Pre-compute R inverse after costs are adapted
		RmInverse_ = Rm_.ldlt().solve(input_matrix_t::Identity());

		// constraint type 1 coefficients
		if (ncEqStateInput_ == 0) {
			DmDager_.setZero();
			EvProjected_.setZero();
			CmProjected_.setZero();
			DmProjected_.setZero();

			AmConstrained_ = Am_;
			QmConstrained_ = Qm_;
			QvConstrained_ = Qv_;
			if (settings_.useRiccatiSolver_==true) {
				RmConstrained_ = Rm_;
			} else {
				BmConstrained_ = Bm_;
				PmConstrained_ = Pm_;
				RvConstrained_ = Rv_;
			}

		} else {
			typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> dynamic_matrix_t;

	    dynamic_matrix_t Cm = Cm_.topRows(ncEqStateInput_);
	    dynamic_matrix_t Dm = Dm_.topRows(ncEqStateInput_);

			// check numerical stability_
			if (checkNumericalStability_==true && ncEqStateInput_>0)
				if (Dm.colPivHouseholderQr().rank()!=ncEqStateInput_) {
					BASE::printString(">>> WARNING: The state-input constraints are rank deficient "
							"(at time " + std::to_string(time) + ")!");
				}

	    dynamic_matrix_t RmInvDmtranspose = RmInverse_*Dm.transpose();
			dynamic_matrix_t RmProjected = ( Dm * RmInvDmtranspose ).ldlt().solve(dynamic_matrix_t::Identity(ncEqStateInput_,ncEqStateInput_));
			dynamic_matrix_t DmDager = RmInvDmtranspose * RmProjected;

			DmDager_.leftCols(ncEqStateInput_) = DmDager;
			EvProjected_.noalias() = DmDager * Ev_.head(ncEqStateInput_);
			CmProjected_.noalias() = DmDager * Cm;
			DmProjected_.noalias() = DmDager * Dm;

	    AmConstrained_ = Am_;
	    AmConstrained_.noalias() -=	Bm_*CmProjected_;

	    state_matrix_t PmTransDmDagerCm = Pm_.transpose()*CmProjected_;
	    QmConstrained_ = Qm_ - PmTransDmDagerCm - PmTransDmDagerCm.transpose();
	    QmConstrained_.noalias() +=	Cm.transpose()*RmProjected*Cm;

	    QvConstrained_ = Qv_;
	    QvConstrained_.noalias() -= CmProjected_.transpose()*Rv_;

	    input_matrix_t DmNullSpaceProjection = input_matrix_t::Identity() - DmProjected_;
	    if (settings_.useRiccatiSolver_==true) {
				RmConstrained_.noalias() = DmNullSpaceProjection.transpose() * Rm_ * DmNullSpaceProjection;
			} else {
				BmConstrained_.noalias() = Bm_ * DmNullSpaceProjection;
				PmConstrained_.noalias() = DmNullSpaceProjection.transpose() * Pm_;
				RvConstrained_.noalias() = DmNullSpaceProjection.transpose() * Rv_;
			}

		}

		// making sure that constrained Qm is PSD
		if (settings_.useMakePSD_==true)
			makePSD(QmConstrained_);
	}



private:
	const char* algorithmName_;

	std::unique_ptr<derivatives_base_t> systemDerivativesPtr_;
	std::unique_ptr<constraint_base_t> systemConstraintsPtr_;
	std::unique_ptr<cost_function_base_t> costFunctionPtr_;

	state_matrix_t       Am_;
	state_input_matrix_t Bm_;

	size_t                     ncEqStateInput_;  	// nc1: Number of the Type-1  active constraints
	constraint1_vector_t       Ev_;
	constraint1_state_matrix_t Cm_;
	constraint1_input_matrix_t Dm_;

	size_t                     ncEqStateOnly_;  // nc2: Number of the Type-2 active constraints
	constraint2_vector_t       Hv_;
	constraint2_state_matrix_t Fm_;

	size_t                     ncIneq_;  // ncIneq: Number of inequality constraints
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


	state_matrix_t AmConstrained_;
	state_matrix_t QmConstrained_;
	state_vector_t QvConstrained_;
	input_matrix_t RmConstrained_;
	control_constraint1_matrix_t DmDager_;
	input_vector_t       EvProjected_;  // DmDager * Ev
	input_state_matrix_t CmProjected_;  // DmDager * Cm
	input_matrix_t       DmProjected_;  // DmDager * Dm
	state_input_matrix_t BmConstrained_;
	input_state_matrix_t PmConstrained_;
	input_vector_t       RvConstrained_;

//	std::vector<std::shared_ptr<penalty_base_t>> penaltyPtrStock_;
//	scalar_t nominalInequalityConstraintPenalty_;
//	scalar_t nominalInequalityConstraintISE_;


	const bool checkNumericalStability_ = true;

};

} // namespace ocs2

#endif /* LINEAR_QUADRATIC_APPROXIMATOR_OCS2_H_ */
