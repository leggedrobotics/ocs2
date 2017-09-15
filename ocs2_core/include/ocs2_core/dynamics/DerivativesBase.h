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
 * Base class for the linearized system dynamics and constraints Derivatives. The linearized system dynamics
 * and constraints are defined as: \n
 * - Linearized system:                        \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$ \n
 * - Linearized state-input constraints:       \f$ C(t) \delta x + D(t) \delta u + e(t) = 0 \f$ \n
 * - Linearized only-state constraints:        \f$ F(t) \delta x + h(t) = 0 \f$ \n
 * - Linearized only-state final constraints:  \f$ F_f(t) \delta x + h_f(t) = 0 \f$
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
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

	/**
	 * Default constructor
	 */
	DerivativesBase() {}

	/**
	 * Default destructor
	 */
	virtual ~DerivativesBase() {}

	/**
	 * Sets the current time, state, and control inout.
	 *
	 * @param [in] t: Current time.
	 * @param [in] x: Current state.
	 * @param [in] u: Current input.
	 */
	virtual void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x,
			const control_vector_t& u) {
		t_ = t;
		x_ = x;
		u_ = u;
	}

	/**
	 * Initializes the system derivative.
	 *
	 * @param [in] systemStockIndexes: The subsystem-stock index vector
	 * @param [in] switchingTimes: The switching time vector.
	 * @param [in] initState: Initial state.
	 * @param [in] activeSubsystemIndex: Current active subsystem index.
	 * @param [in] algorithmName: The algorithm that uses this class.
	 */
	virtual void initializeModel(const std::vector<size_t>& systemStockIndexes, const std::vector<scalar_t>& switchingTimes,
			const state_vector_t& initState, const size_t& activeSubsystemIndex=0, const char* algorithmName=NULL)
	{
		if (activeSubsystemIndex>=switchingTimes.size()-1)
			throw std::runtime_error("activeSubsystemIndex refers to a non-existing subsystem based on the input switchingTimes sequence.");
	}

	/**
	 * The A matrix at a given operating point for the linearized system,
	 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
	 *
	 * @param [out] A: \f$ A(t) \f$ matrix.
	 */
	virtual void getDerivativeState(state_matrix_t& A) = 0;

	/**
	 * The B matrix at a given operating point for the linearized system,
	 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
	 *
	 * @param [out] B: \f$ B(t) \f$ matrix.
	 */
	virtual void getDerivativesControl(control_gain_matrix_t& B) = 0;

	/**
	 * The C matrix at a given operating point for the linearized state-input constraints,
	 * \f$ C(t) \delta x + D(t) \delta u + e(t) = 0 \f$.
	 *
	 * @param [out] C: \f$ C(t) \f$ matrix.
	 */
	virtual void getConstraint1DerivativesState(constraint1_state_matrix_t& C) {}

	/**
	 * The D matrix at a given operating point for the linearized state-input constraints,
	 * \f$ C(t) \delta x + D(t) \delta u + e(t) = 0 \f$.
	 *
	 * @param [out] D: \f$ D(t) \f$ matrix.
	 */
	virtual void getConstraint1DerivativesControl(constraint1_control_matrix_t& D) {}

	/**
	 * The F matrix at a given operating point for the linearized state-only constraints,
	 * \f$ F(t) \delta x + h(t) = 0 \f$.
	 *
	 * @param [out] F: \f$ F(t) \f$ matrix.
	 */
	virtual void getConstraint2DerivativesState(constraint2_state_matrix_t& F) {}

	/**
	 * The F matrix at a given operating point for the linearized terminal state-only constraints,
	 * \f$ F_f(t) \delta x + h_f(t) = 0 \f$.
	 *
	 * @param [out] F_f: \f$ F_f(t) \f$ matrix.
	 */
	virtual void getFinalConstraint2DerivativesState(constraint2_state_matrix_t& F_f) {}

	/**
	 * Returns pointer to DerivativesBase class.
	 *
	 * @return a shared_ptr pointer.
	 */
	virtual std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM> > clone() const = 0;

protected:
	scalar_t t_;
	state_vector_t x_;
	control_vector_t u_;
};

} // namespace ocs2

#endif /* DERIVATIVESBASE_OCS2_H_ */
