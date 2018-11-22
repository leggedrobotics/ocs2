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

#ifndef COSTFUNCTIONBASE_OCS2_H_
#define COSTFUNCTIONBASE_OCS2_H_

#include <memory>

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/misc/LinearInterpolation.h"
#include "ocs2_core/logic/rules/LogicRulesBase.h"
#include "ocs2_core/logic/rules/NullLogicRules.h"
#include "ocs2_core/logic/machine/LogicRulesMachine.h"
#include "ocs2_core/cost/CostDesiredTrajectories.h"

namespace ocs2{

/**
 * Cost Function Base.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules>
class CostFunctionBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	static_assert(std::is_base_of<LogicRulesBase, LOGIC_RULES_T>::value,
			"LOGIC_RULES_T must inherit from LogicRulesBase");

	enum
	{
		state_dim_ = STATE_DIM,
		input_dim_ = INPUT_DIM
	};

	typedef LOGIC_RULES_T logic_rules_t;

	typedef std::shared_ptr<CostFunctionBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > Ptr;
	typedef std::shared_ptr<const CostFunctionBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > ConstPtr;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;

	typedef typename DIMENSIONS::scalar_t                scalar_t;
	typedef typename DIMENSIONS::scalar_array_t          scalar_array_t;
	typedef typename DIMENSIONS::state_vector_t          state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t    state_vector_array_t;
	typedef typename DIMENSIONS::state_matrix_t          state_matrix_t;
	typedef typename DIMENSIONS::input_vector_t          input_vector_t;
	typedef typename DIMENSIONS::input_vector_array_t    input_vector_array_t;
	typedef typename DIMENSIONS::input_matrix_t          input_matrix_t;
	typedef typename DIMENSIONS::input_state_matrix_t    input_state_matrix_t;
	typedef typename DIMENSIONS::state_input_matrix_t    state_input_matrix_t;
	typedef typename DIMENSIONS::dynamic_vector_t        dynamic_vector_t;
	typedef typename DIMENSIONS::dynamic_vector_array_t  dynamic_vector_array_t;

	typedef CostDesiredTrajectories<scalar_t> cost_desired_trajectories_t;

	/**
	 * Default constructor
	 */
	CostFunctionBase()
	: costDesiredTrajectoriesPtr_(nullptr)
	, xNominalFunc_()
	, uNominalFunc_()
	{
		xNominalFunc_.setZero();
		uNominalFunc_.setZero();
	};

	/**
	 * Copy constructor
	 */
	CostFunctionBase(const CostFunctionBase& rhs)
	: CostFunctionBase()
	{}

	/**
	 * Default destructor
	 */
	virtual ~CostFunctionBase() = default;

	/**
	 * Sets the desired state and input trajectories used in the cost function.
	 *
	 * @param [in] CostDesiredTrajectories: cost desired trajectories interface class.
	 */
	virtual void setCostDesiredTrajectories(
			const cost_desired_trajectories_t& costDesiredTrajectories);

	/**
	 * Gets the desired state and input trajectories used in the cost function.
	 *
	 * @param [in] CostDesiredTrajectories: cost desired trajectories interface class.
	 */
	virtual void getCostDesiredTrajectories(
			cost_desired_trajectories_t& costDesiredTrajectories) const;

    /**
     * Returns pointer to the class.
     *
     * @return A raw pointer to the class.
     */
	virtual CostFunctionBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>* clone() const = 0;

	/**
	 * Initializes the cost function.
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
			const char* algorithmName = nullptr)
	{}

    /**
     * Sets the current time, state, and control input
     *
     * @param [in] t: Current time
     * @param [in] x: Current state vector
     * @param [in] u: Current input vector
     */
	virtual void setCurrentStateAndControl(
			const scalar_t& t,
			const state_vector_t& x,
			const input_vector_t& u) {

		t_ = t;
		x_ = x;
		u_ = u;
	}

    /**
     * Get the intermediate cost.
     *
     * @param [out] L: The intermediate cost value.
     */
	virtual void getIntermediateCost(scalar_t& L) = 0;

	/**
	 * Get the time derivative of the intermediate cost.
	 *
	 * @param [out] dLdt: The time derivative of intermediate cost.
	 */
	virtual void getIntermediateCostDerivativeTime(scalar_t& dLdt) { dLdt = 0; }

    /**
     * Get the state derivative of the intermediate cost.
     *
     * @param [out] dLdx: First order derivative of the intermediate cost with respect to state vector.
     */
	virtual void getIntermediateCostDerivativeState(state_vector_t& dLdx) = 0;

    /**
     * Get state second order derivative of the intermediate cost.
     *
     * @param [out] dLdxx: Second order derivative of the intermediate cost with respect to state vector.
     */
	virtual void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) = 0;

    /**
     * Get control input derivative of the intermediate cost.
     *
     * @param [out] dLdu: First order derivative of the intermediate cost with respect to input vector.
     */
	virtual void getIntermediateCostDerivativeInput(input_vector_t& dLdu) = 0;

    /**
     * Get control input second derivative of the intermediate cost.
     *
     * @param [out] dLduu: Second order derivative of the intermediate cost with respect to input vector.
     */
	virtual void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) = 0;

    /**
     * Get the input-state derivative of the intermediate cost.
     *
     * @param [out] dLdux: Second order derivative of the intermediate cost with respect to input vector and state.
     */
	virtual void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdux) = 0;

    /**
     * Get the terminal cost.
     *
     * @param [out] Phi: The final cost value.
     */
	virtual void getTerminalCost(scalar_t& Phi) = 0;

	/**
	 * Get the time derivative of terminal cost.
	 *
	 * @param [out] dPhidt: The time derivative of terminal cost.
	 */
	virtual void getTerminalCostDerivativeTime(scalar_t& dPhidt) { dPhidt = 0; }

    /**
     * Get the terminal cost state derivative of the terminal cost.
     *
     * @param [out] dPhidx: First order final cost derivative with respect to state vector.
     */
	virtual void getTerminalCostDerivativeState(state_vector_t& dPhidx) = 0;

    /**
     * Get the terminal cost state second derivative of the terminal cost.
     *
     * @param [out] dPhidxx: Second order final cost derivative with respect to state vector.
     */
	virtual void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) = 0;

protected:

/*
 * Variables
 */
	const cost_desired_trajectories_t* costDesiredTrajectoriesPtr_;

	typename cost_desired_trajectories_t::dynamic_linear_interpolation_t xNominalFunc_;
	typename cost_desired_trajectories_t::dynamic_linear_interpolation_t uNominalFunc_;

	scalar_t t_;
	state_vector_t x_;
	input_vector_t u_;
};

} // namespace ocs2

#include "implementation/CostFunctionBase.h"

#endif /* COSTFUNCTIONBASE_OCS2_H_ */
