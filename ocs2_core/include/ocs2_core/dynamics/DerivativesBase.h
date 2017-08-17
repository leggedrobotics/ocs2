/*
 * DerivativesBase.h
 *
 *  Created on: Jan 3, 2016
 *      Author: farbod
 */

#ifndef DERIVATIVESBASE_OCS2_H_
#define DERIVATIVESBASE_OCS2_H_

#include <memory>
#include <cstring>

#include "ocs2_core/Dimensions.h"


namespace ocs2{

/**
 * Base class for Derivative
 * @tparam STATE_DIM
 * @tparam INPUT_DIM
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class DerivativesBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::scalar_t scalar_t;
	typedef typename DIMENSIONS::state_vector_t   state_vector_t;
	typedef typename DIMENSIONS::control_vector_t control_vector_t;
	typedef typename DIMENSIONS::output_vector_t  output_vector_t;
	typedef typename DIMENSIONS::state_matrix_t state_matrix_t;
	typedef typename DIMENSIONS::control_gain_matrix_t control_gain_matrix_t;
	typedef typename DIMENSIONS::constraint1_state_matrix_t   constraint1_state_matrix_t;
	typedef typename DIMENSIONS::constraint1_control_matrix_t constraint1_control_matrix_t;
	typedef typename DIMENSIONS::constraint2_state_matrix_t   constraint2_state_matrix_t;

	DerivativesBase() {}
	virtual ~DerivativesBase() {}

	/**
	 * Sets current state and control
	 * @param [in] t
	 * @param [in] x
	 * @param [in] u
	 */
	virtual void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x,
			const control_vector_t& u) {
		t_ = t;
		x_ = x;
		u_ = u;
	}

	/**
	 * Initializes the model
	 * @param [in] systemStockIndexes
	 * @param [in] switchingTimes
	 * @param [in] initState
	 * @param [in] activeSubsystemIndex
	 * @param [in] algorithmName
	 */
	virtual void initializeModel(const std::vector<size_t>& systemStockIndexes, const std::vector<scalar_t>& switchingTimes,
			const state_vector_t& initState, const size_t& activeSubsystemIndex=0, const char* algorithmName=NULL)
	{
		if (activeSubsystemIndex>=switchingTimes.size()-1)
			throw std::runtime_error("activeSubsystemIndex refers to a non-existing subsystem based on the input switchingTimes sequence.");
	}

	/**
	 *
	 * @param [out] A
	 */
	virtual void getDerivativeState(state_matrix_t& A) = 0;  //FIXME: consistentcy in naming

	/**
	 *
	 * @param [out] B
	 */
	virtual void getDerivativesControl(control_gain_matrix_t& B) = 0;

	/**
	 *
	 * @param [out] C
	 */
	virtual void getConstraint1DerivativesState(constraint1_state_matrix_t& C) {}

	/**
	 *
	 * @param [out] D
	 */
	virtual void getConstraint1DerivativesControl(constraint1_control_matrix_t& D) {}

	/**
	 *
	 * @param [out] F
	 */
	virtual void getConstraint2DerivativesState(constraint2_state_matrix_t& F) {}

	/**
	 *
	 * @param [out] F
	 */
	virtual void getFinalConstraint2DerivativesState(constraint2_state_matrix_t& F) {}

	/**
	 *
	 * @return
	 */
	virtual std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM> > clone() const = 0;

protected:
	scalar_t t_;
	state_vector_t x_;
	control_vector_t u_;
};

} // namespace ocs2

#endif /* DERIVATIVESBASE_OCS2_H_ */
