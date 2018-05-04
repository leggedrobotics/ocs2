/*
 * LinearConstraintAD.h
 *
 *  Created on: May 3, 2018
 *      Author: farbod
 */

#ifndef LINEARCONSTRAINTAD_OCS2_H_
#define LINEARCONSTRAINTAD_OCS2_H_

#include "ocs2_core/constraint/ConstraintBaseAD.h"

namespace ocs2{

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=ocs2::NullLogicRules<STATE_DIM,INPUT_DIM>>
class LinearConstraintAD : public
ConstraintBaseAD<LinearConstraintAD<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<LinearConstraintAD<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > Ptr;
	typedef std::shared_ptr<const LinearConstraintAD<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > ConstPtr;

	typedef ConstraintBaseAD<LinearConstraintAD<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>, STATE_DIM, INPUT_DIM> BASE;
	typedef typename BASE::scalar_t 				scalar_t;
	typedef typename BASE::state_vector_t   		state_vector_t;
	typedef typename BASE::input_vector_t	 		input_vector_t;
	typedef typename BASE::state_matrix_t   		state_matrix_t;
	typedef typename BASE::control_gain_matrix_t 	control_gain_matrix_t;
	typedef typename BASE::constraint1_vector_t 	constraint1_vector_t;
	typedef typename BASE::constraint2_vector_t 	constraint2_vector_t;
	typedef typename BASE::constraint1_state_matrix_t   constraint1_state_matrix_t;
	typedef typename BASE::constraint1_control_matrix_t constraint1_control_matrix_t;
	typedef typename BASE::constraint2_state_matrix_t   constraint2_state_matrix_t;

	LinearConstraintAD(
			const size_t& numStateInputConstraint,
			const constraint1_vector_t& e,
			const constraint1_state_matrix_t& C,
			const constraint1_control_matrix_t& D,
			const size_t& numStateOnlyConstraint,
			const constraint2_vector_t& h,
			const constraint2_state_matrix_t& F,
			const size_t& numStateOnlyFinalConstraint,
			const constraint2_vector_t& h_f,
			const constraint2_state_matrix_t& F_f)

		: numStateInputConstraint_(numStateInputConstraint)
		, e_(e)
		, C_(C)
		, D_(D)
		, numStateOnlyConstraint_(numStateOnlyConstraint)
		, h_(h)
		, F_(F)
		, numStateOnlyFinalConstraint_(numStateOnlyFinalConstraint)
		, h_f_(h_f)
		, F_f_(F_f)
	{}

	~LinearConstraintAD() = default;

	/**
	 * Interface method to the state-input equality constriants. This method should be implemented by the derived class.
	 *
	 * @tparam scalar type. All the floating point operations should be with this type.
	 * @param [in] time: time.
	 * @param [in] state: state vector.
	 * @param [in] input: input vector
	 * @param [out] constraintVector: constriant vecotr.
	 */
	template <typename SCALAR_T>
	void stateInputConstraint(
			const SCALAR_T& time,
			const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
			const Eigen::Matrix<SCALAR_T, INPUT_DIM, 1>& input,
			Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& constraintVector) {

		constraintVector = e_.template cast<SCALAR_T>()
				+ C_.template cast<SCALAR_T>() * state
				+ D_.template cast<SCALAR_T>() * input;
	}

	/**
	 * Get the number of state-input active equality constriants.
	 *
	 * @param [in] time: time.
	 * @return number of state-input active equality constriants.
	 */
	virtual size_t numStateInputConstraint(const scalar_t& time) override {

		return numStateInputConstraint_;
	}

	/**
	 * Interface method to the state-only equality constriants. This method should be implemented by the derived class.
	 *
	 * @tparam scalar type. All the floating point operations should be with this type.
	 * @param [in] time: time.
	 * @param [in] state: state vector.
	 * @param [out] constraintVector: constriant vecotr.
	 */
	template <typename SCALAR_T>
	void stateOnlyConstraint(
			const SCALAR_T& time,
			const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
			Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& constraintVector) {

		constraintVector = h_.template cast<SCALAR_T>()
				+ F_.template cast<SCALAR_T>() * state;
	}

	/**
	 * Get the number of state-only active equality constriants.
	 *
	 * @param [in] time: time.
	 * @return number of state-only active equality constriants.
	 */
	virtual size_t numStateOnlyConstraint(const scalar_t& time) override {

		return numStateOnlyConstraint_;
	}

	/**
	 * Interface method to the state-only final equality constriants. This method should be implemented by the derived class.
	 *
	 * @tparam scalar type. All the floating point operations should be with this type.
	 * @param [in] time: time.
	 * @param [in] state: state vector.
	 * @param [out] constraintVector: constriant vecotr.
	 */
	template <typename SCALAR_T>
	void stateOnlyFinalConstraint(
			const SCALAR_T& time,
			const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
			Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& constraintVector) {

		constraintVector = h_f_.template cast<SCALAR_T>()
				+ F_f_.template cast<SCALAR_T>() * state;
	}

	/**
	 * Get the number of final state-only active equality constriants.
	 *
	 * @param [in] time: time.
	 * @return number of final state-only active equality constriants.
	 */
	virtual size_t numStateOnlyFinalConstraint(const scalar_t& time) override {

		return numStateOnlyFinalConstraint_;
	}

private:
	size_t numStateInputConstraint_;
	constraint1_vector_t 			e_;
	constraint1_state_matrix_t 		C_;
	constraint1_control_matrix_t 	D_;
	size_t numStateOnlyConstraint_;
	constraint2_vector_t 			h_;
	constraint2_state_matrix_t 		F_;
	size_t numStateOnlyFinalConstraint_;
	constraint2_vector_t 			h_f_;
	constraint2_state_matrix_t 		F_f_;
};

} // namespace ocs2

#endif /* LINEARCONSTRAINTAD_OCS2_H_ */
