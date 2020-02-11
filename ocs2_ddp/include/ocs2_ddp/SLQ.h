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

#pragma once

#include <ocs2_core/integration/Integrator.h>
#include <ocs2_core/integration/SystemEventHandler.h>

#include "ocs2_ddp/DDP_BASE.h"
#include "ocs2_ddp/SLQ_Settings.h"
#include "ocs2_ddp/riccati_equations/ContinuousTimeRiccatiEquations.h"

namespace ocs2 {

/**
 * This class is an interface class for the single-thread and multi-thread SLQ.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class SLQ final : public DDP_BASE<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = DDP_BASE<STATE_DIM, INPUT_DIM>;

  using typename BASE::DIMENSIONS;
  using typename BASE::dynamic_matrix_array2_t;
  using typename BASE::dynamic_matrix_array_t;
  using typename BASE::dynamic_matrix_t;
  using typename BASE::dynamic_vector_array2_t;
  using typename BASE::dynamic_vector_array_t;
  using typename BASE::dynamic_vector_t;
  using typename BASE::input_matrix_array2_t;
  using typename BASE::input_matrix_array_t;
  using typename BASE::input_matrix_t;
  using typename BASE::input_vector_array2_t;
  using typename BASE::input_vector_array_t;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_array2_t;
  using typename BASE::scalar_array_t;
  using typename BASE::scalar_t;
  using typename BASE::size_array2_t;
  using typename BASE::size_array_t;
  using typename BASE::state_matrix_array2_t;
  using typename BASE::state_matrix_array_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_array2_t;
  using typename BASE::state_vector_array_t;
  using typename BASE::state_vector_t;

  using typename BASE::controller_array_t;
  using typename BASE::controller_ptr_array_t;
  using typename BASE::controller_t;
  using typename BASE::linear_controller_array_t;
  using typename BASE::linear_controller_ptr_array_t;
  using typename BASE::linear_controller_t;

  using typename BASE::constraint_base_t;
  using typename BASE::cost_function_base_t;
  using typename BASE::derivatives_base_t;
  using typename BASE::event_handler_t;
  using typename BASE::operating_trajectories_base_t;
  using typename BASE::rollout_base_t;

  using riccati_equations_t = ContinuousTimeRiccatiEquations<STATE_DIM, INPUT_DIM>;
  using s_vector_t = typename riccati_equations_t::s_vector_t;
  using s_vector_array_t = typename riccati_equations_t::s_vector_array_t;

  /**
   * Default constructor.
   */
  SLQ() = default;

  /**
   * Constructor
   *
   * @param [in] rolloutPtr: The rollout class used for simulating the system dynamics.
   * @param [in] systemDerivativesPtr: The system dynamics derivatives for subsystems of the system.
   * @param [in] systemConstraintsPtr: The system constraint function and its derivatives for subsystems.
   * @param [in] costFunctionPtr: The cost function (intermediate and terminal costs) and its derivatives for subsystems.
   * @param [in] operatingTrajectoriesPtr: The operating trajectories of system which will be used for initialization of SLQ.
   * @param [in] settings: Structure containing the settings for the SLQ algorithm.
   * @param [in] logicRulesPtr: The logic rules used for implementing mixed-logic dynamical systems.
   * @param [in] heuristicsFunctionPtr: Heuristic function used in the infinite time optimal control formulation. If it is not
   * defined, we will use the terminal cost function defined in costFunctionPtr.
   */
  SLQ(const rollout_base_t* rolloutPtr, const derivatives_base_t* systemDerivativesPtr, const constraint_base_t* systemConstraintsPtr,
      const cost_function_base_t* costFunctionPtr, const operating_trajectories_base_t* operatingTrajectoriesPtr,
      const SLQ_Settings& settings = SLQ_Settings(), std::shared_ptr<HybridLogicRules> logicRulesPtr = nullptr,
      const cost_function_base_t* heuristicsFunctionPtr = nullptr);

  /**
   * Default destructor.
   */
  ~SLQ() override = default;

  /**
   * Gets a reference to the Options structure.
   *
   * @return a reference to the Options structure.
   */
  SLQ_Settings& settings();

 protected:
  void approximateIntermediateLQ(const scalar_array_t& timeTrajectory, const size_array_t& postEventIndices,
                                 const state_vector_array_t& stateTrajectory, const input_vector_array_t& inputTrajectory,
                                 ModelDataBase::array_t& modelDataTrajectory) override;

  void calculateControllerWorker(size_t workerIndex, size_t partitionIndex, size_t timeIndex) override;

  scalar_t solveSequentialRiccatiEquations(const state_matrix_t& SmFinal, const state_vector_t& SvFinal, const scalar_t& sFinal) override;

  dynamic_matrix_t computeHamiltonianHessian(DDP_Strategy strategy, const ModelDataBase& modelData,
                                             const dynamic_matrix_t& Sm) const override;

  void riccatiEquationsWorker(size_t workerIndex, size_t partitionIndex, const dynamic_matrix_t& SmFinal, const dynamic_vector_t& SvFinal,
                              const scalar_t& sFinal) override;

  /**
   * Integrates the riccati equation and generates the value function at the times set in nominal Time Trajectory.
   *
   * @param riccatiIntegrator [in] : Riccati integrator object
   * @param riccatiEquation [in] : Riccati equation object
   * @param nominalTimeTrajectory [in] : time trajectory produced in the forward rollout.
   * @param nominalEventsPastTheEndIndices [in] : Indices into nominalTimeTrajectory to point to times right after event times
   * @param allSsFinal [in] : Final value of the value function.
   * @param SsNormalizedTime [out] : Time trajectory of the value function.
   * @param SsNormalizedPostEventIndices [out] : Indices into SsNormalizedTime to point to times right after event times
   * @param allSsTrajectory [out] : Value function in vector format.
   */
  void integrateRiccatiEquationNominalTime(IntegratorBase<riccati_equations_t::S_DIM_>& riccatiIntegrator,
                                           riccati_equations_t& riccatiEquation, const scalar_array_t& nominalTimeTrajectory,
                                           const size_array_t& nominalEventsPastTheEndIndices, s_vector_t allSsFinal,
                                           scalar_array_t& SsNormalizedTime, size_array_t& SsNormalizedPostEventIndices,
                                           s_vector_array_t& allSsTrajectory);

  /**
   * Integrates the riccati equation and freely selects the time nodes for the value function.
   *
   * @param riccatiIntegrator [in] : Riccati integrator object
   * @param riccatiEquation [in] : Riccati equation object
   * @param nominalTimeTrajectory [in] : time trajectory produced in the forward rollout.
   * @param nominalEventsPastTheEndIndices [in] : Indices into nominalTimeTrajectory to point to times right after event times
   * @param allSsFinal [in] : Final value of the value function.
   * @param SsNormalizedTime [out] : Time trajectory of the value function.
   * @param SsNormalizedPostEventIndices [out] : Indices into SsNormalizedTime to point to times right after event times
   * @param allSsTrajectory [out] : Value function in vector format.
   */
  void integrateRiccatiEquationAdaptiveTime(IntegratorBase<riccati_equations_t::S_DIM_>& riccatiIntegrator,
                                            riccati_equations_t& riccatiEquation, const scalar_array_t& nominalTimeTrajectory,
                                            const size_array_t& nominalEventsPastTheEndIndices, s_vector_t allSsFinal,
                                            scalar_array_t& SsNormalizedTime, size_array_t& SsNormalizedPostEventIndices,
                                            s_vector_array_t& allSsTrajectory);

  /****************
   *** Variables **
   ****************/
  SLQ_Settings settings_;

  std::vector<std::shared_ptr<riccati_equations_t>> riccatiEquationsPtrStock_;
  std::vector<std::unique_ptr<IntegratorBase<riccati_equations_t::S_DIM_>>> riccatiIntegratorPtrStock_;
};

}  // namespace ocs2

#include "implementation/SLQ.h"
