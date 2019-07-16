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

#ifndef QUADRATICCOSTFUNCTIONAD_OCS2_H_
#define QUADRATICCOSTFUNCTIONAD_OCS2_H_

#include "ocs2_core/cost/CostFunctionBaseAD.h"

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules, size_t LOGIC_VARIABLE_DIM=0>
class QuadraticCostFunctionAD : public
CostFunctionBaseAD<QuadraticCostFunctionAD<STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM>, STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<QuadraticCostFunctionAD<STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM> > Ptr;
	typedef std::shared_ptr<const QuadraticCostFunctionAD<STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM> > ConstPtr;

	typedef CostFunctionBaseAD<
			QuadraticCostFunctionAD<STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM>,
			STATE_DIM, INPUT_DIM, LOGIC_RULES_T, LOGIC_VARIABLE_DIM> BASE;
	typedef typename BASE::scalar_t             scalar_t;
	typedef typename BASE::scalar_array_t       scalar_array_t;
	typedef typename BASE::state_vector_t       state_vector_t;
	typedef typename BASE::state_vector_array_t state_vector_array_t;
	typedef typename BASE::state_matrix_t       state_matrix_t;
	typedef typename BASE::input_vector_t       input_vector_t;
	typedef typename BASE::input_vector_array_t input_vector_array_t;
	typedef typename BASE::input_matrix_t       input_matrix_t;
	typedef typename BASE::input_state_matrix_t input_state_matrix_t;
	typedef typename BASE::logic_variable_t     logic_variable_t;

	QuadraticCostFunctionAD(
			const state_matrix_t& Q,
			const input_matrix_t& R,
			const state_vector_t& xNominal,
			const input_vector_t& uNominal,
			const state_matrix_t& QFinal,
			const input_state_matrix_t& P = input_state_matrix_t::Zero())
	: Q_(Q)
	, R_(R)
	, P_(P)
	, QFinal_(QFinal)
	, xNominal_(xNominal)
	, uNominal_(uNominal)
	{}

	/**
	 * Default destructor.
	 */
	virtual ~QuadraticCostFunctionAD() = default;

	/**
	 * Interface method to the intermediate cost function. This method should be implemented by the derived class.
	 *
	 * @tparam scalar type. All the floating point operations should be with this type.
	 * @param [in] time: time.
	 * @param [in] state: state vector.
	 * @param [in] input: input vector.
	 * @param [in] stateDesired: desired state vector.
	 * @param [in] inputDesired: desired input vector.
	 * @param [in] logicVariable: logic variable vector.
	 * @param [out] costValue: cost value.
	 */
	template <typename SCALAR_T>
	void intermediateCostFunction(
			const SCALAR_T& time,
			const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
			const Eigen::Matrix<SCALAR_T, INPUT_DIM, 1>& input,
			const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& stateDesired,
			const Eigen::Matrix<SCALAR_T, INPUT_DIM, 1>& inputDesired,
			const Eigen::Matrix<SCALAR_T, LOGIC_VARIABLE_DIM, 1>& logicVariable,
			SCALAR_T& costValue) {

		Eigen::Matrix<SCALAR_T, STATE_DIM, 1> xDeviation = state - stateDesired;
		Eigen::Matrix<SCALAR_T, INPUT_DIM, 1> uDeviation = input - inputDesired;

		costValue = 0.5 * xDeviation.dot(Q_.template cast<SCALAR_T>() * xDeviation)
				+ 0.5 * uDeviation.dot(R_.template cast<SCALAR_T>() * uDeviation)
				+ uDeviation.dot(P_.template cast<SCALAR_T>() * xDeviation);;
	}

	/**
	 * Interface method to the terminal cost function. This method should be implemented by the derived class.
	 *
	 * @tparam scalar type. All the floating point operations should be with this type.
	 * @param [in] time: time.
	 * @param [in] state: state vector.
	 * @param [in] stateDesired: desired state vector.
	 * @param [in] logicVariable: logic variable vector.
	 * @param [out] costValue: cost value.
	 */
	template <typename SCALAR_T>
	void terminalCostFunction(
			const SCALAR_T& time,
			const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
			const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& stateDesired,
			const Eigen::Matrix<SCALAR_T, LOGIC_VARIABLE_DIM, 1>& logicVariable,
			SCALAR_T& costValue) {

		Eigen::Matrix<SCALAR_T, STATE_DIM, 1> xDeviation = state - stateDesired;
		costValue = 0.5 * xDeviation.dot(QFinal_.template cast<SCALAR_T>() * xDeviation);
	}

	/**
	 * Gets a user-defined desired state at the give time.
	 *
	 * @param [in] t: Current time.
	 * @return The desired state at the give time.
	 */
	state_vector_t getDesiredState(const scalar_t& t) {

		return xNominal_;
	}

	/**
	 * Gets a user-defined desired input at the give time.
	 *
	 * @param [in] t: Current time.
	 * @return The desired input at the give time.
	 */
	input_vector_t getDesiredInput(const scalar_t& t) {

		return uNominal_;
	}

	/**
	 * Gets a user-defined logic variable based on the given logic rules.
	 *
	 * @param [in] logicRulesMachine: A class which contains and parse the logic rules e.g
	 * method findActiveSubsystemHandle returns a Lambda expression which can be used to
	 * find the ID of the current active subsystem.
	 * @param [in] partitionIndex: index of the time partition.
	 * @param [in] algorithmName: The algorithm that class this class (default not defined).
	 * @return The the logic variables.
	 */
	logic_variable_t getlogicVariables(scalar_t time) {

		return logic_variable_t::Zero();
	}

private:
	state_matrix_t Q_;
	input_matrix_t R_;
	input_state_matrix_t P_;
	state_matrix_t QFinal_;

	state_vector_t xNominal_;
	input_vector_t uNominal_;
};

} // namespace ocs2

#endif /* QUADRATICCOSTFUNCTIONAD_OCS2_H_ */
