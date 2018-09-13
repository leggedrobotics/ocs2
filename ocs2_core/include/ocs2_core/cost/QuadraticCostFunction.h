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

#ifndef QUADRATICCOSTFUNCTION_OCS2_H_
#define QUADRATICCOSTFUNCTION_OCS2_H_

#include "ocs2_core/cost/CostFunctionBase.h"

namespace ocs2{

/**
 * Quadratic Cost Function.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules>
class QuadraticCostFunction : public CostFunctionBase< STATE_DIM, INPUT_DIM, LOGIC_RULES_T>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef CostFunctionBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> BASE;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::scalar_t scalar_t;
	typedef typename DIMENSIONS::state_vector_t state_vector_t;
	typedef typename DIMENSIONS::state_matrix_t state_matrix_t;
	typedef typename DIMENSIONS::input_vector_t input_vector_t;
	typedef typename DIMENSIONS::input_matrix_t input_matrix_t;
	typedef typename DIMENSIONS::input_state_matrix_t input_state_matrix_t;

	/**
	 * Constructor for the running and final cost function defined as the following:
	 * - \f$ L = 0.5(x-x_{n})' Q (x-x_{n}) + 0.5(u-u_{n})' R (u-u_{n}) + (u-u_{n})' P (x-x_{n}) \f$
	 * - \f$ \Phi = 0.5(x-x_{f})' Q_{f} (x-x_{f}) \f$.
	 * @param [in] Q: \f$ Q \f$
	 * @param [in] R: \f$ R \f$
	 * @param [in] xNominalIntermediate: \f$ x_{n}\f$
	 * @param [in] uNominalIntermediate: \f$ u_{n}\f$
	 * @param [in] xNominalFinal: \f$ x_{f}\f$
	 * @param [in] QFinal: \f$ Q_{f}\f$
	 */
	QuadraticCostFunction(
			const state_matrix_t& Q,
			const input_matrix_t& R,
			const state_vector_t& xNominalIntermediate,
			const input_vector_t& uNominalIntermediate,
			const state_matrix_t& QFinal,
			const state_vector_t& xNominalFinal,
			const input_state_matrix_t& P = input_state_matrix_t::Zero())
	: Q_(Q)
	, R_(R)
	, P_(P)
	, QFinal_(QFinal)
	, xNominalIntermediate_(xNominalIntermediate)
	, uNominalIntermediate_(uNominalIntermediate)
	, xNominalFinal_(xNominalFinal)
	{}

	/**
	 * Destructor
	 */
	virtual ~QuadraticCostFunction() = default;

	/**
	 * Initializes the quadratic cost function.
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
			const char* algorithmName=NULL) override {

		BASE::initializeModel(logicRulesMachine, partitionIndex, algorithmName);
	}

    /**
     * Returns pointer to the class.
     *
     * @return A raw pointer to the class.
     */
	virtual QuadraticCostFunction* clone() const override {

		return new QuadraticCostFunction<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>(*this);
	}

	/**
	 * Sets the current time, state, and control input.
	 *
	 * @param [in] t: Current time.
	 * @param [in] x: Current state vector.
	 * @param [in] u: Current input vector.
	 */
	virtual void setCurrentStateAndControl(
			const scalar_t& t,
			const state_vector_t& x,
			const input_vector_t& u) override {

		setCurrentStateAndControl(t, x, u,
				xNominalIntermediate_, uNominalIntermediate_, xNominalFinal_);
	}

	/**
	 * Sets the current time, state, control input, and desired state and input.
	 *
	 * @param [in] t: Current time.
	 * @param [in] x: Current state vector.
	 * @param [in] u: Current input vector.
	 * @param [in] xNominalIntermediate: Intermediate desired state vector.
	 * @param [in] uNominalIntermediate: Intermediate desired input vector.
	 * @param [in] xNominalFinal: Final desired state vector.
	 */
	virtual void setCurrentStateAndControl(
			const scalar_t& t,
			const state_vector_t& x,
			const input_vector_t& u,
			const state_vector_t& xNominalIntermediate,
			const input_vector_t& uNominalIntermediate,
			const state_vector_t& xNominalFinal) {

		BASE::setCurrentStateAndControl(t, x, u);

		xNominalIntermediate_ = xNominalIntermediate;
		uNominalIntermediate_ = uNominalIntermediate;
		xNominalFinal_ = xNominalFinal;
		xIntermediateDeviation_ = x - xNominalIntermediate;
		uIntermediateDeviation_ = u - uNominalIntermediate;
	}

    /**
     * Get the intermediate cost.
     *
     * @param [out] L: The intermediate cost value.
     */
	virtual void getIntermediateCost(scalar_t& L) override {

		L = 0.5 * xIntermediateDeviation_.dot(Q_ * xIntermediateDeviation_) +
				0.5 * uIntermediateDeviation_.dot(R_ * uIntermediateDeviation_) +
				uIntermediateDeviation_.dot(P_ * xIntermediateDeviation_);
	}

    /**
     * Get the state derivative of the intermediate cost.
     *
     * @param [out] dLdx: First order derivative of the intermediate cost with respect to state vector.
     */
	virtual void getIntermediateCostDerivativeState(state_vector_t& dLdx) override {

		dLdx =  Q_ * xIntermediateDeviation_ + P_.transpose() * uIntermediateDeviation_;
	}

    /**
     * Get state second order derivative of the intermediate cost.
     *
     * @param [out] dLdxx: Second order derivative of the intermediate cost with respect to state vector.
     */
	virtual void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) override {

		dLdxx = Q_;
	}

    /**
     * Get control input derivative of the intermediate cost.
     *
     * @param [out] dLdu: First order derivative of the intermediate cost with respect to input vector.
     */
	virtual void getIntermediateCostDerivativeInput(input_vector_t& dLdu) override {

		dLdu = R_ * uIntermediateDeviation_ + P_ * xIntermediateDeviation_;
	}

    /**
     * Get control input second derivative of the intermediate cost.
     *
     * @param [out] dLduu: Second order derivative of the intermediate cost with respect to input vector.
     */
	virtual void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) override {

		dLduu = R_;
	}

    /**
     * Get the input-state derivative of the intermediate cost.
     *
     * @param [out] dLdux: Second order derivative of the intermediate cost with respect to input vector and state.
     */
	virtual void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdux) override {

		dLdux = P_;
	}

    /**
     * Get the terminal cost.
     *
     * @param [out] cost: The final cost value.
     */
	virtual void getTerminalCost(scalar_t& cost) override {

		state_vector_t xFinalDeviation = BASE::x_ - xNominalFinal_;
		cost = 0.5 * xFinalDeviation.dot(QFinal_ * xFinalDeviation);
	}

    /**
     * Get the terminal cost state derivative.
     *
     * @param [out] dPhidx: First order final cost derivative with respect to state vector.
     */
	virtual void getTerminalCostDerivativeState(state_vector_t& dPhidx) override {

		state_vector_t xFinalDeviation = BASE::x_ - xNominalFinal_;
		dPhidx = QFinal_ * xFinalDeviation;
	}

    /**
     * Get the terminal cost state second derivative
     *
     * @param [out] dPhidxx: Second order final cost derivative with respect to state vector.
     */
	virtual void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) override {

		dPhidxx = QFinal_;
	}

protected:
	state_matrix_t Q_;
	input_matrix_t R_;
	input_state_matrix_t P_;
	state_matrix_t QFinal_;

	state_vector_t xNominalIntermediate_;
	input_vector_t uNominalIntermediate_;

	state_vector_t xIntermediateDeviation_;
	input_vector_t uIntermediateDeviation_;

	state_vector_t xNominalFinal_;
};

} // namespace ocs2

#endif /* QUADRATICCOSTFUNCTION_OCS2_H_ */
