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

#ifndef SYSTEMDYNAMICSBASE_OCS2_H_
#define SYSTEMDYNAMICSBASE_OCS2_H_

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/dynamics/ControlledSystemBase.h"
#include "ocs2_core/dynamics/DerivativesBase.h"
#include "ocs2_core/logic/rules/LogicRulesBase.h"
#include "ocs2_core/logic/rules/NullLogicRules.h"
#include "ocs2_core/logic/machine/LogicRulesMachine.h"

namespace ocs2{

/**
 * The system dynamics interface.
 * The linearized system flow map is defined as: \n
 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$ \n
 * The linearized system jump map is defined as: \n
 * \f$ x^+ = G \delta x + H \delta u \f$ \n
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM, size_t NUM_MODES=1>
class SystemDynamicsBase
		: public DerivativesBase<STATE_DIM, INPUT_DIM>
		, public ControlledSystemBase<STATE_DIM, INPUT_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum
	{
		num_modes_	= NUM_MODES,
		state_dim_ 	= STATE_DIM,
		input_dim_ 	= INPUT_DIM,
	};

	using Ptr = std::shared_ptr<SystemDynamicsBase<STATE_DIM, INPUT_DIM> >;
	using ConstPtr = std::shared_ptr<const SystemDynamicsBase<STATE_DIM, INPUT_DIM> >;

	using DEV_BASE = DerivativesBase<STATE_DIM, INPUT_DIM> 	;
	using DYN_BASE = ControlledSystemBase<STATE_DIM, INPUT_DIM> ;

	using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
	using scalar_t = typename DIMENSIONS::scalar_t;
	using state_vector_t = typename DIMENSIONS::state_vector_t;
	using input_vector_t = typename DIMENSIONS::input_vector_t;
	using state_matrix_t = typename DIMENSIONS::state_matrix_t;
	using state_input_matrix_t = typename DIMENSIONS::state_input_matrix_t;
	using dynamic_vector_t = typename DIMENSIONS::dynamic_vector_t;
	using dynamic_state_matrix_t = typename DIMENSIONS::dynamic_state_matrix_t;
	using dynamic_input_matrix_t = typename DIMENSIONS::dynamic_input_matrix_t;

	/**
	 * Default constructor
	 */
	SystemDynamicsBase() {
		if (NUM_MODES==0) {
			throw std::runtime_error("The system should have at least one mode.");
		}
	}

	/**
	 * Copy constructor
	 */
	SystemDynamicsBase(const SystemDynamicsBase& rhs) = default;

	/**
	 * Default destructor
	 */
	virtual ~SystemDynamicsBase() = default;

    /**
     * Returns pointer to the class.
     *
     * @return A raw pointer to the class.
     */
	virtual SystemDynamicsBase<STATE_DIM, INPUT_DIM>* clone() const = 0;

	/**
	 * Interface method to the state flow map of the hybrid system.
	 *
	 * @param [in] time: time.
	 * @param [in] state: state vector.
	 * @param [in] input: input vector
	 * @param [out] stateDerivative: state vector time derivative.
	 */
	virtual void computeFlowMap(
			const scalar_t& time,
			const state_vector_t& state,
			const input_vector_t& input,
			state_vector_t& stateDerivative) = 0;

	/**
	 * Interface method to the state jump map of the hybrid system.
	 *
	 * @param [in] time: time.
	 * @param [in] state: state vector.
	 * @param [out] jumpedState: jumped state.
	 */
	virtual void computeJumpMap(
			const scalar_t& time,
			const state_vector_t& state,
			state_vector_t& jumpedState)  {

		DYN_BASE::computeJumpMap(time, state, jumpedState);
	}

	/**
	 * Interface method to the guard surfaces.
	 *
	 * @param [in] time: transition time
	 * @param [in] state: transition state
	 * @param [out] guardSurfacesValue: A vector of guard surfaces values
	 */
	virtual void computeGuardSurfaces(
			const scalar_t& time,
			const state_vector_t& state,
			dynamic_vector_t& guardSurfacesValue) {

		DYN_BASE::computeGuardSurfaces(time, state, guardSurfacesValue);
	}


    /**
     * Sets the current time, state, and control input.
     *
     * @param [in] time: time
     * @param [in] state: state vector
     * @param [in] input: input vector
     */
	virtual void setCurrentStateAndControl(
			const scalar_t& time,
			const state_vector_t& state,
			const input_vector_t& input) {

		DEV_BASE::setCurrentStateAndControl(time, state, input);
	}


	/**
	 * Partial time derivative of the system flow map.
	 * \f$ \frac{\partial f}{\partial t}  \f$.
	 *
	 * @param [out] df: \f$ \frac{\partial f}{\partial t} \f$ matrix.
	 */
	virtual void getFlowMapDerivativeTime(state_vector_t& df) {

		DEV_BASE::getFlowMapDerivativeTime(df);
	}

	/**
	 * The A matrix at a given operating point for the linearized system flow map.
	 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
	 *
	 * @param [out] A: \f$ A(t) \f$ matrix.
	 */
	virtual void getFlowMapDerivativeState(state_matrix_t& A) = 0;

	/**
	 * The B matrix at a given operating point for the linearized system flow map.
	 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
	 *
	 * @param [out] B: \f$ B(t) \f$ matrix.
	 */
	virtual void getFlowMapDerivativeInput(state_input_matrix_t& B) = 0;


	/**
	 * Partial time derivative of the system jump map.
	 * \f$ \frac{\partial g}{\partial t}  \f$.
	 *
	 * @param [out] dg: \f$ \frac{\partial g}{\partial t} \f$ matrix.
	 */
	virtual void getJumpMapDerivativeTime(state_vector_t& dg) {

		DEV_BASE::getJumpMapDerivativeTime(dg);
	}

	/**
	 * The G matrix at a given operating point for the linearized system jump map.
	 * \f$ x^+ = G \delta x + H \delta u \f$.
	 *
	 * @param [out] G: \f$ G \f$ matrix.
	 */
	virtual void getJumpMapDerivativeState(state_matrix_t& G) {

		DEV_BASE::getJumpMapDerivativeState(G);
	}

	/**
	 * The H matrix at a given operating point for the linearized system jump map.
	 * \f$ x^+ = G \delta x + H \delta u \f$.
	 *
	 * @param [out] H: \f$ H \f$ matrix.
	 */
	virtual void getJumpMapDerivativeInput(state_input_matrix_t& H) {

		DEV_BASE::getJumpMapDerivativeInput(H);
	}


	/**
	 * Get at a given operating point the derivative of the guard surfaces w.r.t. input vector.
	 *
	 * @param [out] D_t_gamma: Derivative of the guard surfaces w.r.t. time.
	 */
	virtual void getGuardSurfacesDerivativeTime(dynamic_vector_t& D_t_gamma) {

		DEV_BASE::getGuardSurfacesDerivativeTime(D_t_gamma);
	}

	/**
	 * Get at a given operating point the derivative of the guard surfaces w.r.t. input vector.
	 *
	 * @param [out] D_x_gamma: Derivative of the guard surfaces w.r.t. state vector.
	 */
	virtual void getGuardSurfacesDerivativeState(dynamic_state_matrix_t& D_x_gamma) {

		DEV_BASE::getGuardSurfacesDerivativeState(D_x_gamma);
	}

	/**
	 * Get at a given operating point the derivative of the guard surfaces w.r.t. input vector.
	 *
	 * @param [out] D_u_gamma: Derivative of the guard surfaces w.r.t. input vector.
	 */
	virtual void getGuardSurfacesDerivativeInput(dynamic_input_matrix_t& D_u_gamma) {

		DEV_BASE::getGuardSurfacesDerivativeInput(D_u_gamma);
	}

protected:

};

} // namespace ocs2


#endif /* SYSTEMDYNAMICSBASE_OCS2_H_ */
