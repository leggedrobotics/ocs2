/*
 * LinearSystemDynamicsAD.h
 *
 *  Created on: May 2, 2018
 *      Author: farbod
 */

#ifndef LINEARSYSTEMDYNAMICSAD_OCS2_H_
#define LINEARSYSTEMDYNAMICSAD_OCS2_H_

#include "ocs2_core/dynamics/SystemDynamicsBaseAD.h"

namespace ocs2{

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=ocs2::NullLogicRules<STATE_DIM,INPUT_DIM>>
class LinearSystemDynamicsAD : public
SystemDynamicsBaseAD<LinearSystemDynamicsAD<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<LinearSystemDynamicsAD<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > Ptr;
	typedef std::shared_ptr<const LinearSystemDynamicsAD<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > ConstPtr;

	typedef SystemDynamicsBaseAD<LinearSystemDynamicsAD<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>, STATE_DIM, INPUT_DIM> BASE;
	typedef typename BASE::scalar_t       			scalar_t;
	typedef typename BASE::state_vector_t 			state_vector_t;
	typedef typename BASE::state_matrix_t 			state_matrix_t;
	typedef typename BASE::input_vector_t  			input_vector_t;
	typedef typename BASE::state_input_matrix_t	state_input_matrix_t;

	LinearSystemDynamicsAD(
			const state_matrix_t& A,
			const state_input_matrix_t& B,
			const state_matrix_t& G,
			const state_input_matrix_t& H)
	: A_(A)
	, B_(B)
	, G_(G)
	, H_(H)
	{}

	~LinearSystemDynamicsAD() = default;

	/**
	 * Interface method to the state flow map of the hybrid system. This method should be implemented by the derived class.
	 *
	 * @tparam scalar type. All the floating point operations should be with this type.
	 * @param [in] time: time.
	 * @param [in] state: state vector.
	 * @param [in] input: input vector
	 * @param [out] stateDerivative: state vector time derivative.
	 */
	template <typename SCALAR_T>
	void systemFlowMap(
			const SCALAR_T& time,
			const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
			const Eigen::Matrix<SCALAR_T, INPUT_DIM, 1>& input,
			Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& stateDerivative) {

		stateDerivative =
				A_.template cast<SCALAR_T>() * state +
				B_.template cast<SCALAR_T>() * input;
	}

	/**
	 * Interface method to the state jump map of the hybrid system. This method should be implemented by the derived class.
	 *
	 * @tparam scalar type. All the floating point operations should be with this type.
	 * @param [in] time: time.
	 * @param [in] state: state vector.
	 * @param [out] jumpedState: jumped state.
	 */
	template <typename SCALAR_T>
	void systemJumpMap(
			const SCALAR_T& time,
			const Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& state,
			const Eigen::Matrix<SCALAR_T, INPUT_DIM, 1>& input,
			Eigen::Matrix<SCALAR_T, STATE_DIM, 1>& jumpedState) {

		jumpedState =
				G_.template cast<SCALAR_T>() * state +
				H_.template cast<SCALAR_T>() * input;
	}

private:
	state_matrix_t 			A_;
	state_input_matrix_t 	B_;
	state_matrix_t 			G_;
	state_input_matrix_t 	H_;
};

} // namespace ocs2

#endif /* LINEARSYSTEMDYNAMICSAD_OCS2_H_ */
