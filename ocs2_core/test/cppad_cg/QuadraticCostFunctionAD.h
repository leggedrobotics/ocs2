/*
 * QuadraticCostFunctionAD.h
 *
 *  Created on: Apr 26, 2018
 *      Author: farbod
 */

#ifndef QUADRATICCOSTFUNCTIONAD_OCS2_H_
#define QUADRATICCOSTFUNCTIONAD_OCS2_H_

#include "ocs2_core/cost/CostFunctionBaseAD.h"

namespace ocs2{

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=ocs2::NullLogicRules<STATE_DIM,INPUT_DIM>>
class QuadraticCostFunctionAD : public
CostFunctionBaseAD<QuadraticCostFunctionAD<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<QuadraticCostFunctionAD<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > Ptr;
	typedef std::shared_ptr<const QuadraticCostFunctionAD<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > ConstPtr;

	typedef CostFunctionBaseAD<QuadraticCostFunctionAD<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>, STATE_DIM, INPUT_DIM> BASE;
	typedef typename BASE::scalar_t       			scalar_t;
	typedef typename BASE::scalar_array_t 			scalar_array_t;
	typedef typename BASE::state_vector_t 			state_vector_t;
	typedef typename BASE::state_vector_array_t 	state_vector_array_t;
	typedef typename BASE::state_matrix_t 			state_matrix_t;
	typedef typename BASE::input_vector_t  			input_vector_t;
	typedef typename BASE::input_vector_array_t 	input_vector_array_t;
	typedef typename BASE::control_matrix_t 		control_matrix_t;
	typedef typename BASE::control_feedback_t 		control_feedback_t;

	QuadraticCostFunctionAD(
			const state_matrix_t& Q,
			const control_matrix_t& R,
			const state_vector_t& x_nominal,
			const input_vector_t& u_nominal,
			const state_vector_t& x_final,
			const state_matrix_t& Q_final)
	: Q_(Q)
	, R_(R)
	, x_nominal_(x_nominal)
	, u_nominal_(u_nominal)
	, x_final_(x_final)
	, Q_final_(Q_final)
	{}

	virtual ~QuadraticCostFunctionAD() = default;

	template <typename SCALAR_T>
	void intermediateCostFunction(
			const SCALAR_T& time,
			const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
			const Eigen::Matrix<SCALAR_T, INPUT_DIM, 1>& input,
			SCALAR_T& costValue) {

		Eigen::Matrix<SCALAR_T, STATE_DIM, 1> x_deviation = state - x_nominal_.template cast<SCALAR_T>();
		Eigen::Matrix<SCALAR_T, INPUT_DIM, 1> u_deviation = input - u_nominal_.template cast<SCALAR_T>();

		costValue = 0.5 * x_deviation.dot(Q_.template cast<SCALAR_T>() * x_deviation) +
				0.5 * u_deviation.dot(R_.template cast<SCALAR_T>() * u_deviation);
	}

	template <typename SCALAR_T>
	void terminalCostFunction(
			const SCALAR_T& time,
			const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
			SCALAR_T& costValue) {

		Eigen::Matrix<SCALAR_T, STATE_DIM, 1> x_deviation = state - x_final_.template cast<SCALAR_T>();
		costValue = 0.5 * x_deviation.dot(Q_final_.template cast<SCALAR_T>() * x_deviation);
	}

private:
	state_vector_t x_deviation_;
	state_vector_t x_nominal_;
	state_matrix_t Q_;

	input_vector_t u_deviation_;
	input_vector_t u_nominal_;
	control_matrix_t R_;

	state_vector_t x_final_;
	state_matrix_t Q_final_;
};

} // namespace ocs2

#endif /* QUADRATICCOSTFUNCTIONAD_OCS2_H_ */
