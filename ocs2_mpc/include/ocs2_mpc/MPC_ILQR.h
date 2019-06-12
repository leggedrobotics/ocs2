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

#ifndef MPC_ILQR_OCS2_H_
#define MPC_ILQR_OCS2_H_

#include <ocs2_core/Dimensions.h>
#include <ocs2_ddp_base/DDP_BASE.h>

#include <ocs2_ilqr/ILQR_BASE.h>
#include <ocs2_ilqr/ILQR_ST.h>
#include <ocs2_ilqr/ILQR_MT.h>

#include "ocs2_mpc/MPC_BASE.h"

namespace ocs2{

/**
 * This an MPC implementation with ILQR optimal control solver.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules>
class MPC_ILQR : public MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> BASE;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::scalar_t                     scalar_t;
	typedef typename DIMENSIONS::scalar_array_t               scalar_array_t;
	typedef typename DIMENSIONS::scalar_array2_t              scalar_array2_t;
	typedef typename DIMENSIONS::size_array_t                 size_array_t;
	typedef typename DIMENSIONS::size_array2_t                size_array2_t;
	typedef typename DIMENSIONS::state_vector_t               state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t         state_vector_array_t;
	typedef typename DIMENSIONS::state_vector_array2_t        state_vector_array2_t;
	typedef typename DIMENSIONS::input_vector_t               input_vector_t;
	typedef typename DIMENSIONS::input_vector_array_t         input_vector_array_t;
	typedef typename DIMENSIONS::input_vector_array2_t        input_vector_array2_t;
	typedef typename DIMENSIONS::input_state_matrix_t         input_state_matrix_t;
	typedef typename DIMENSIONS::input_state_matrix_array_t   input_state_matrix_array_t;
	typedef typename DIMENSIONS::input_state_matrix_array2_t  input_state_matrix_array2_t;
	typedef typename DIMENSIONS::dynamic_vector_t             dynamic_vector_t;
	typedef typename DIMENSIONS::dynamic_vector_array_t       dynamic_vector_array_t;

	typedef typename BASE::cost_desired_trajectories_t cost_desired_trajectories_t;
	typedef typename BASE::mode_sequence_template_t    mode_sequence_template_t;
	typedef typename BASE::controller_ptr_array_t      controller_ptr_array_t;

	typedef LinearController<STATE_DIM,INPUT_DIM> linear_controller_t;
	typedef typename linear_controller_t::array_t linear_controller_array_t;

	typedef ocs2::DDP_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> ddp_base_t;

	typedef ocs2::ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> ilqr_base_t;
	typedef ocs2::ILQR_ST<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>   ilqr_t;
	typedef ocs2::ILQR_MT<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>   ilqr_mp_t;

	typedef typename ddp_base_t::logic_rules_machine_t         logic_rules_machine_t;
	typedef typename ddp_base_t::controlled_system_base_t      controlled_system_base_t;
	typedef typename ddp_base_t::event_handler_t               event_handler_t;
	typedef typename ddp_base_t::derivatives_base_t            derivatives_base_t;
	typedef typename ddp_base_t::constraint_base_t             constraint_base_t;
	typedef typename ddp_base_t::cost_function_base_t          cost_function_base_t;
	typedef typename ddp_base_t::operating_trajectories_base_t operating_trajectories_base_t;

	/**
	 * Default constructor.
	 */
	MPC_ILQR();

	/**
	 * Constructor
	 *
	 * @param [in] systemDynamicsPtr: The system dynamics which possibly includes some subsystems.
	 * @param [in] systemDerivativesPtr: The system dynamics derivatives for subsystems of the system.
	 * @param [in] systemConstraintsPtr: The system constraint function and its derivatives for subsystems.
	 * @param [in] costFunctionPtr: The cost function (intermediate and terminal costs) and its derivatives for subsystems.
	 * @param [in] operatingTrajectoriesPtr: The operating trajectories of system which will be used for initialization of ILQR.
	 * @param [in] partitioningTimes: This will be used as the initial time partitioning. As the MPC progresses the internal
	 * partitioningTimes will be shifted in time automatically.
	 * @param [in] ilqrSettings: Structure containing the settings for the ILQR algorithm.
	 * @param [in] mpcSettings: Structure containing the settings for the MPC algorithm.
	 * @param [in] logicRulesPtr: The logic rules used for implementing mixed logical dynamical systems.
	 * @param [in] heuristicsFunctionPtr: Heuristic function used in the infinite time optimal control formulation. If it is not
	 * defined, we will use the terminal cost function defined in costFunctionPtr.
	 */
	MPC_ILQR(const controlled_system_base_t* systemDynamicsPtr,
			const derivatives_base_t* systemDerivativesPtr,
			const constraint_base_t* systemConstraintsPtr,
			const cost_function_base_t* costFunctionPtr,
			const operating_trajectories_base_t* operatingTrajectoriesPtr,
			const scalar_array_t& partitioningTimes,
			const ILQR_Settings& ilqrSettings = ILQR_Settings(),
			const MPC_Settings& mpcSettings = MPC_Settings(),
			const LOGIC_RULES_T* logicRulesPtr = nullptr,
			const mode_sequence_template_t* modeSequenceTemplatePtr = nullptr,
			const cost_function_base_t* heuristicsFunctionPtr = nullptr);

	/**
	 * Default destructor.
	 */
	virtual ~MPC_ILQR() = default;

	/**
	 * Gets the ILQR settings structure.
	 *
	 * @return ILQR settings structure
	 */
	virtual ILQR_Settings& ilqrSettings();

	/**
	 * Gets a pointer to the underlying solver used in the MPC.
	 *
	 * @return A pointer to the underlying solver used in the MPC
	 */
	virtual ilqr_base_t* getSolverPtr() override;

	/**
	 * Solves the optimal control problem for the given state and time period ([initTime,finalTime]).
	 *
	 * @param [out] initTime: Initial time.
	 * @param [in] initState: Initial state.
	 * @param [out] finalTime: Final time.
	 * @param [out] timeTrajectoriesStock: A pointer to the optimized time trajectories.
	 * @param [out] stateTrajectoriesStock: A pointer to the optimized state trajectories.
	 * @param [out] inputTrajectoriesStock: A pointer to the optimized input trajectories.
	 * @param [out] controllerStock_out: A pointer to the optimized control policy.
	 */
	virtual void calculateController(
			const scalar_t& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const scalar_array2_t*& timeTrajectoriesStockPtr,
			const state_vector_array2_t*& stateTrajectoriesStockPtr,
			const input_vector_array2_t*& inputTrajectoriesStockPtr,
			const controller_ptr_array_t*& controllerStockPtr) override;

protected:

	/***********
	 * Variables
	 ***********/
	std::unique_ptr<ilqr_base_t> ilqrPtr_;

	scalar_array2_t       optimizedTimeTrajectoriesStock_;
	state_vector_array2_t optimizedStateTrajectoriesStock_;
	input_vector_array2_t optimizedInputTrajectoriesStock_;

};

} // namespace ocs2

#include "implementation/MPC_ILQR.h"

#endif /* MPC_ILQR_OCS2_H_ */