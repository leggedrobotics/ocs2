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

#ifndef SLQ_OCS2_H_
#define SLQ_OCS2_H_


#include "ocs2_slq/SLQ_BASE.h"


namespace ocs2{

/**
 * This class implements single thread SLQ algorithm.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
  */
template <size_t STATE_DIM, size_t INPUT_DIM>
class SLQ : public SLQ_BASE<STATE_DIM, INPUT_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef SLQ_BASE<STATE_DIM, INPUT_DIM> BASE;

	using DIMENSIONS = typename BASE::DIMENSIONS;

	using lagrange_t = typename DIMENSIONS::template LinearFunction_t<Eigen::Dynamic>;
	using size_array_t = typename DIMENSIONS::size_array_t;
	using size_array2_t = typename DIMENSIONS::size_array2_t;
	using scalar_t = typename DIMENSIONS::scalar_t;
	using scalar_array_t = typename DIMENSIONS::scalar_array_t;
	using scalar_array2_t = typename DIMENSIONS::scalar_array2_t;
	using eigen_scalar_t = typename DIMENSIONS::eigen_scalar_t;
	using eigen_scalar_array_t = typename DIMENSIONS::eigen_scalar_array_t;
	using eigen_scalar_array2_t = typename DIMENSIONS::eigen_scalar_array2_t;
	using state_vector_t = typename DIMENSIONS::state_vector_t;
	using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
	using state_vector_array2_t = typename DIMENSIONS::state_vector_array2_t;
	using input_vector_t = typename DIMENSIONS::input_vector_t;
	using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
	using input_vector_array2_t = typename DIMENSIONS::input_vector_array2_t;
	using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t;
	using input_state_matrix_array_t = typename DIMENSIONS::input_state_matrix_array_t;
	using input_state_matrix_array2_t = typename DIMENSIONS::input_state_matrix_array2_t;
	using state_matrix_t = typename DIMENSIONS::state_matrix_t;
	using state_matrix_array_t = typename DIMENSIONS::state_matrix_array_t;
	using state_matrix_array2_t = typename DIMENSIONS::state_matrix_array2_t;
	using input_matrix_t = typename DIMENSIONS::input_matrix_t;
	using input_matrix_array_t = typename DIMENSIONS::input_matrix_array_t;
	using input_matrix_array2_t = typename DIMENSIONS::input_matrix_array2_t;
	using state_input_matrix_t = typename DIMENSIONS::state_input_matrix_t;
	using state_input_matrix_array_t = typename DIMENSIONS::state_input_matrix_array_t;
	using state_input_matrix_array2_t = typename DIMENSIONS::state_input_matrix_array2_t;
	using constraint1_vector_t = typename DIMENSIONS::constraint1_vector_t;
	using constraint1_vector_array_t = typename DIMENSIONS::constraint1_vector_array_t;
	using constraint1_vector_array2_t = typename DIMENSIONS::constraint1_vector_array2_t;
	using constraint1_state_matrix_t = typename DIMENSIONS::constraint1_state_matrix_t;
	using constraint1_state_matrix_array_t = typename DIMENSIONS::constraint1_state_matrix_array_t;
	using constraint1_state_matrix_array2_t = typename DIMENSIONS::constraint1_state_matrix_array2_t;
	using constraint1_input_matrix_t = typename DIMENSIONS::constraint1_input_matrix_t;
	using constraint1_input_matrix_array_t = typename DIMENSIONS::constraint1_input_matrix_array_t;
	using constraint1_input_matrix_array2_t = typename DIMENSIONS::constraint1_input_matrix_array2_t;
	using input_constraint1_matrix_t = typename DIMENSIONS::input_constraint1_matrix_t;
	using input_constraint1_matrix_array_t = typename DIMENSIONS::input_constraint1_matrix_array_t;
	using input_constraint1_matrix_array2_t = typename DIMENSIONS::input_constraint1_matrix_array2_t;
	using constraint2_vector_t = typename DIMENSIONS::constraint2_vector_t;
	using constraint2_vector_array_t = typename DIMENSIONS::constraint2_vector_array_t;
	using constraint2_vector_array2_t = typename DIMENSIONS::constraint2_vector_array2_t;
	using constraint2_state_matrix_t = typename DIMENSIONS::constraint2_state_matrix_t;
	using constraint2_state_matrix_array_t = typename DIMENSIONS::constraint2_state_matrix_array_t;
	using constraint2_state_matrix_array2_t = typename DIMENSIONS::constraint2_state_matrix_array2_t;

    using linear_controller_t = typename BASE::linear_controller_t;
    using linear_controller_array_t = typename BASE::linear_controller_array_t;

	using controlled_system_base_t = typename BASE::controlled_system_base_t;
	using event_handler_t = typename BASE::event_handler_t;
	using derivatives_base_t = typename BASE::derivatives_base_t;
	using constraint_base_t = typename BASE::constraint_base_t;
	using cost_function_base_t = typename BASE::cost_function_base_t;
	using operating_trajectories_base_t = typename BASE::operating_trajectories_base_t;

	/**
	 * Default constructor.
	 */
	SLQ()
	: BASE()
	{}

	/**
	 * Constructor
	 *
	 * @param [in] systemDynamicsPtr: The system dynamics which possibly includes some subsystems.
	 * @param [in] systemDerivativesPtr: The system dynamics derivatives for subsystems of the system.
	 * @param [in] systemConstraintsPtr: The system constraint function and its derivatives for subsystems.
	 * @param [in] costFunctionPtr: The cost function (intermediate and terminal costs) and its derivatives for subsystems.
	 * @param [in] operatingTrajectoriesPtr: The operating trajectories of system which will be used for initialization of SLQ.
	 * @param [in] settings: Structure containing the settings for the SLQ algorithm.
	 * @param [in] logicRulesPtr: The logic rules used for implementing mixed logical dynamical systems.
	 * @param [in] heuristicsFunctionPtr: Heuristic function used in the infinite time optimal control formulation. If it is not
	 * defined, we will use the terminal cost function defined in costFunctionPtr.
	 */
	SLQ(const controlled_system_base_t* systemDynamicsPtr,
		const derivatives_base_t* systemDerivativesPtr,
		const constraint_base_t* systemConstraintsPtr,
		const cost_function_base_t* costFunctionPtr,
		const operating_trajectories_base_t* operatingTrajectoriesPtr,
		const SLQ_Settings& settings = SLQ_Settings(),
		std::shared_ptr<HybridLogicRules> logicRulesPtr = nullptr,
		const cost_function_base_t* heuristicsFunctionPtr = nullptr);

	/**
	 * Default destructor.
	 */
	~SLQ();

	/**
	 * Line search on the feedforwrd parts of the controller. It uses the following approach for line search:
	 * The constraint TYPE-1 correction term is directly added through a user defined stepSize (defined in settings_.constraintStepSize_).
	 * But the cost minimization term is optimized through a line-search strategy defined in SLQ settings.
	 *
	 * @param [in] computeISEs: Whether lineSearch needs to calculate ISEs indeces for type_1 and type-2 constraints.
	 */
	void lineSearch(bool computeISEs) override;

	/**
	 * Solves Riccati equations for all the partitions.
	 *
	 * @param [in] SmFinal: The final Sm for Riccati equation.
	 * @param [in] SvFinal: The final Sv for Riccati equation.
	 * @param [in] sFinal: The final s for Riccati equation.
	 *
	 * @return average time step
	 */
	scalar_t solveSequentialRiccatiEquations(
			const state_matrix_t& SmFinal,
			const state_vector_t& SvFinal,
			const eigen_scalar_t& sFinal) override;

	/**
	 * Runs the initialization method for single thread SLQ.
	 */
	void runInit() override;

	/**
	 * Runs a single iteration of single thread SLQ.
	 */
	void runIteration() override;

	/**
	 * Runs the exit method single thread SLQ.
	 */
	void runExit() override;

protected:
	/**
	 * Computes the linearized dynamics for a particular time partition
	 * @param [in] sysIndex
	 */
	void approximatePartitionLQ(const size_t& partitionIndex) override;

	/**
	 * Computes the controller for a particular time partition
	 *
	 * @param partitionIndex: Time partition index
	 */
	void calculatePartitionController(const size_t& partitionIndex) override;


private:


public:
	template <size_t GSLQP_STATE_DIM, size_t GSLQP_INPUT_DIM>
	friend class GSLQP;
};

} // namespace ocs2

#include "implementation/SLQ.h"


#endif /* SLQ_OCS2_H_ */
