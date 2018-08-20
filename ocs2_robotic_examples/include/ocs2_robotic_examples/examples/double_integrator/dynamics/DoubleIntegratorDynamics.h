/*!
 * @author    David Hoeller
 * @email     dhoeller@ethz.ch
 * @author    Farbod Farshidian
 * @email     farshidian@ethz.ch
 *
 * Copyright (C) 2018 Robotic Systems Lab, ETH Zurich.
 * All rights reserved.
 * http://www.rsl.ethz.ch/
 */

#ifndef DOUBLE_INTEGRATOR_DYNAMICS_H
#define DOUBLE_INTEGRATOR_DYNAMICS_H

#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include "ocs2_robotic_examples/examples/double_integrator/definitions.h"

namespace ocs2{

class DoubleIntegratorDynamics : public ControlledSystemBase<double_integrator::STATE_DIM_, double_integrator::INPUT_DIM_>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<DoubleIntegratorDynamics> Ptr;
	typedef std::shared_ptr<const DoubleIntegratorDynamics> ConstPtr;

	typedef NullLogicRules<double_integrator::STATE_DIM_, double_integrator::INPUT_DIM_> logic_rules_t;

	typedef ControlledSystemBase<double_integrator::STATE_DIM_, double_integrator::INPUT_DIM_> BASE;
	typedef typename BASE::scalar_t               scalar_t;
	typedef typename BASE::state_vector_t         state_vector_t;
	typedef typename BASE::input_vector_t         input_vector_t;

	typedef Dimensions<double_integrator::STATE_DIM_, double_integrator::INPUT_DIM_> DIMENSIONS;
	typedef typename DIMENSIONS::state_matrix_t 		state_matrix_t;
	typedef typename DIMENSIONS::state_input_matrix_t 	state_input_matrix_t;


	state_matrix_t A_;
	state_input_matrix_t B_;

	DoubleIntegratorDynamics(state_matrix_t A, state_input_matrix_t B)
	: A_(A)
	, B_(B)
	{}

	~DoubleIntegratorDynamics() = default;

	/**
	 * Returns pointer to the class.
	 *
	 * @return A raw pointer to the class.
	 */
	virtual DoubleIntegratorDynamics* clone() const {

		return new DoubleIntegratorDynamics(*this);
	}

	/**
	 * TODO
	 *
	 * @param [in] time: time.
	 * @param [in] state: state vector.
	 * @param [in] input: input vector
	 * @param [out] stateDerivative: state vector time derivative.
	 */
	void computeFlowMap(
			const scalar_t& time,
			const state_vector_t& state,
			const input_vector_t& input,
			state_vector_t& stateDerivative) {

		stateDerivative = A_*state + B_*input;
	}


};

} // namespace ocs2

#endif //DOUBLE_INTEGRATOR_DYNAMICS_H
