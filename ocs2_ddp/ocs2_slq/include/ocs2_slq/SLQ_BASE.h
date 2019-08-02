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

#ifndef SLQ_BASE_OCS2_H_
#define SLQ_BASE_OCS2_H_

#include <ocs2_ddp_base/DDP_BASE.h>

#include <ocs2_core/integration/Integrator.h>
#include <ocs2_core/integration/StateTriggeredEventHandler.h>
#include <ocs2_core/integration/SystemEventHandler.h>
#include <ocs2_core/misc/LinearAlgebra.h>

#include <ocs2_oc/rollout/StateTriggeredRollout.h>

#include <ocs2_slq/SLQ_Settings.h>

#include <ocs2_slq/riccati_equations/SequentialErrorEquationNormalized.h>
#include <ocs2_slq/riccati_equations/SequentialRiccatiEquationsNormalized.h>

namespace ocs2 {

/**
 * This class is an interface class for the single-thread and multi-thread SLQ.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class SLQ_BASE : public DDP_BASE<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = DDP_BASE<STATE_DIM, INPUT_DIM>;

  using DIMENSIONS = typename BASE::DIMENSIONS;
  using typename BASE::constraint1_input_matrix_array2_t;
  using typename BASE::constraint1_input_matrix_array_t;
  using typename BASE::constraint1_input_matrix_t;
  using typename BASE::constraint1_state_matrix_array2_t;
  using typename BASE::constraint1_state_matrix_array_t;
  using typename BASE::constraint1_state_matrix_t;
  using typename BASE::constraint1_vector_array2_t;
  using typename BASE::constraint1_vector_array_t;
  using typename BASE::constraint1_vector_t;
  using typename BASE::constraint2_state_matrix_array2_t;
  using typename BASE::constraint2_state_matrix_array_t;
  using typename BASE::constraint2_state_matrix_t;
  using typename BASE::constraint2_vector_array2_t;
  using typename BASE::constraint2_vector_array_t;
  using typename BASE::constraint2_vector_t;
  using typename BASE::controller_t;
  using typename BASE::dynamic_matrix_array2_t;
  using typename BASE::dynamic_matrix_t;
  using typename BASE::dynamic_vector_array_t;
  using typename BASE::dynamic_vector_t;
  using typename BASE::eigen_scalar_array2_t;
  using typename BASE::eigen_scalar_array_t;
  using typename BASE::eigen_scalar_t;
  using typename BASE::input_constraint1_matrix_array2_t;
  using typename BASE::input_constraint1_matrix_array_t;
  using typename BASE::input_constraint1_matrix_t;
  using typename BASE::input_matrix_array2_t;
  using typename BASE::input_matrix_array3_t;
  using typename BASE::input_matrix_array_t;
  using typename BASE::input_matrix_t;
  using typename BASE::input_state_matrix_array2_t;
  using typename BASE::input_state_matrix_array3_t;
  using typename BASE::input_state_matrix_array_t;
  using typename BASE::input_state_matrix_t;
  using typename BASE::input_vector_array2_t;
  using typename BASE::input_vector_array3_t;
  using typename BASE::input_vector_array_t;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_array2_t;
  using typename BASE::scalar_array3_t;
  using typename BASE::scalar_array_t;
  using typename BASE::scalar_t;
  using typename BASE::size_array2_t;
  using typename BASE::size_array_t;
  using typename BASE::state_input_matrix_array2_t;
  using typename BASE::state_input_matrix_array3_t;
  using typename BASE::state_input_matrix_array_t;
  using typename BASE::state_input_matrix_t;
  using typename BASE::state_matrix_array2_t;
  using typename BASE::state_matrix_array3_t;
  using typename BASE::state_matrix_array_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_array2_t;
  using typename BASE::state_vector_array3_t;
  using typename BASE::state_vector_array_t;
  using typename BASE::state_vector_t;

  using typename BASE::constraint_base_t;
  using typename BASE::controlled_system_base_t;
  using typename BASE::controller_ptr_array_t;
  using typename BASE::cost_desired_trajectories_t;
  using typename BASE::cost_function_base_t;
  using typename BASE::derivatives_base_t;
  using typename BASE::event_handler_t;
  using typename BASE::linear_controller_array_t;
  using typename BASE::linear_controller_t;
  using typename BASE::linear_quadratic_approximator_t;
  using typename BASE::logic_rules_machine_ptr_t;
  using typename BASE::logic_rules_machine_t;
  using typename BASE::operating_trajectorie_rollout_t;
  using typename BASE::operating_trajectories_base_t;
  using typename BASE::penalty_base_t;
  using typename BASE::rollout_base_t;
  using typename BASE::time_triggered_rollout_t;

  using riccati_equations_t = SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>;
  using error_equation_t = SequentialErrorEquationNormalized<STATE_DIM, INPUT_DIM>;
  using s_vector_t = typename riccati_equations_t::s_vector_t;
  using s_vector_array_t = typename riccati_equations_t::s_vector_array_t;

  using state_triggered_rollout_t = StateTriggeredRollout<STATE_DIM, INPUT_DIM>;

  /**
   * class for collecting SLQ data
   */
  template <size_t OTHER_STATE_DIM, size_t OTHER_INPUT_DIM>
  friend class SLQ_DataCollector;

 public:
  /**
   * Default constructor.
   */
  SLQ_BASE() = default;

  /**
   * Constructor
   *
   * @param [in] systemDynamicsPtr: The system dynamics which possibly includes some subsystems.
   * @param [in] systemDerivativesPtr: The system dynamics derivatives for subsystems of the system.
   * @param [in] systemConstraintsPtr: The system constraint function and its derivatives for subsystems.
   * @param [in] costFunctionPtr: The cost function (intermediate and terminal costs) and its derivatives for subsystems.
   * @param [in] operatingTrajectoriesPtr: The operating trajectories of system which will be used for initialization of SLQ.
   * @param [in] settings: Structure containing the settings for the SLQ algorithm.
   * @param [in] logicRulesPtr: The logic rules used for implementing mixed-logic dynamical systems.
   * @param [in] heuristicsFunctionPtr: Heuristic function used in the infinite time optimal control formulation. If it is not
   * defined, we will use the terminal cost function defined in costFunctionPtr.
   */
  SLQ_BASE(const controlled_system_base_t* systemDynamicsPtr, const derivatives_base_t* systemDerivativesPtr,
           const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr,
           const operating_trajectories_base_t* operatingTrajectoriesPtr, const SLQ_Settings& settings = SLQ_Settings(),
           std::shared_ptr<HybridLogicRules> logicRulesPtr = nullptr, const cost_function_base_t* heuristicsFunctionPtr = nullptr);

  /**
   * Default destructor.
   */
  virtual ~SLQ_BASE() = default;

  /**
   * Approximates the nonlinear problem as a linear-quadratic problem around the nominal
   * state and control trajectories. This method updates the following variables:
   * 	- linearized system model and constraints
   * 	- \f$ dxdt = A_m(t)x + B_m(t)u \f$.
   * 	- s.t. \f$ C_m(t)x + D_m(t)u + E_v(t) = 0 \f$ \\
   * 	-      \f$ F_m(t)x + H_v(t) = 0 \f$ .
   * 	- AmTrajectoryStock_: \f$ A_m\f$  matrix.
   * 	- BmTrajectoryStock_: \f$ B_m\f$  matrix.
   * 	- CmTrajectoryStock_: \f$ C_m\f$ matrix.
   * 	- DmTrajectoryStock_: \f$ D_m\f$ matrix.
   * 	- EvTrajectoryStock_: \f$ E_v\f$ vector.
   * 	- FmTrajectoryStock_: \f$ F_m\f$ vector.
   * 	- HvTrajectoryStock_: \f$ H_v\f$ vector.
   *
   * 	- quadratized intermediate cost function
   * 	- intermediate cost: \f$ q(t) + 0.5 xQ_m(t)x + x'Q_v(t) + u'P_m(t)x + 0.5u'R_m(t)u + u'R_v(t) \f$
   * 	- qTrajectoryStock_:  \f$ q\f$
   * 	- QvTrajectoryStock_: \f$ Q_v\f$ vector.
   * 	- QmTrajectoryStock_:\f$  Q_m\f$ matrix.
   * 	- PmTrajectoryStock_: \f$ P_m\f$ matrix.
   * 	- RvTrajectoryStock_: \f$ R_v\f$ vector.
   * 	- RmTrajectoryStock_: \f$ R_m\f$ matrix.
   * 	- RmInverseTrajectoryStock_: inverse of \f$ R_m\f$ matrix.
   *
   * 	- as well as the constrained coefficients of
   * 		- linearized system model
   * 		- quadratized intermediate cost function
   * 		- quadratized final cost
   *
   */
  void approximateOptimalControlProblem() override;

  /**
   * Calculates the controller. This method uses the following variables:
   * - constrained, linearized model
   * - constrained, quadratized cost
   *
   * The method modifies:
   * - nominalControllersStock_: the controller that stabilizes the system around the new nominal trajectory and
   * 				improves the constraints as well as the increment to the feed-forward control input.
   */
  void calculateController() override;

  /**
   * Gets a reference to the Options structure.
   *
   * @return a reference to the Options structure.
   */
  SLQ_Settings& settings();

 protected:
  /**
   * Sets up optimizer for different number of partitions.
   *
   * @param [in] numPartitions: number of partitions.
   */
  virtual void setupOptimizer(size_t numPartitions);

  /**
   * Computes the controller for a particular time partition
   *
   * @param partitionIndex: Time partition index
   */
  virtual void calculatePartitionController(size_t partitionIndex) = 0;

  /**
   * Calculates an LQ approximate of the optimal control problem at a given partition and a node.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] partitionIndex: Time partition index.
   * @param [in] timeIndex: Time index in the partition.
   */
  void approximateLQWorker(size_t workerIndex, size_t partitionIndex, size_t timeIndex) override;

  /**
   * Modify the unconstrained LQ coefficients to constrained ones.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] i: Time partition index.
   * @param [in] k: Time index in the partition.
   * @param [in] stateConstraintPenalty: State-only constraint penalty.
   */
  virtual void approximateConstrainedLQWorker(size_t workerIndex, size_t i, size_t k, scalar_t stateConstraintPenalty);

  /**
   * Calculates controller at a given partition and a node.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] partitionIndex: Time partition index
   * @param [in] timeIndex: Time index in the partition
   */
  void calculateControllerWorker(size_t workerIndex, size_t partitionIndex, size_t timeIndex) override;

  /**
   * Solves a set of Riccati equations and type_1 constraints error correction compensation for the partition in the given index.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] partitionIndex: The requested partition index to solve Riccati equations.
   * @param [in] SmFinal: The final Sm for Riccati equation.
   * @param [in] SvFinal: The final Sv for Riccati equation.
   * @param [in] sFinal: The final s for Riccati equation.
   * @param [in] SveFinal: The final Sve for the current Riccati equation.
   */
  void solveSlqRiccatiEquationsWorker(size_t workerIndex, size_t partitionIndex, const state_matrix_t& SmFinal,
                                      const state_vector_t& SvFinal, const eigen_scalar_t& sFinal, const state_vector_t& SveFinal);

  /****************
   *** Variables **
   ****************/
  SLQ_Settings settings_;

  std::vector<typename rollout_base_t::Ptr> state_dynamicsForwardRolloutPtrStock_;

  state_matrix_array2_t AmConstrainedTrajectoryStock_;
  state_matrix_array2_t QmConstrainedTrajectoryStock_;
  state_vector_array2_t QvConstrainedTrajectoryStock_;
  dynamic_matrix_array2_t RmInvConstrainedCholTrajectoryStock_;
  input_constraint1_matrix_array2_t DmDagerTrajectoryStock_;
  input_vector_array2_t EvProjectedTrajectoryStock_;        // DmDager * Ev
  input_state_matrix_array2_t CmProjectedTrajectoryStock_;  // DmDager * Cm
  input_matrix_array2_t DmProjectedTrajectoryStock_;        // DmDager * Dm
  input_matrix_array2_t RmInverseTrajectoryStock_;

  std::vector<std::shared_ptr<riccati_equations_t>> riccatiEquationsPtrStock_;
  std::vector<std::shared_ptr<SystemEventHandler<riccati_equations_t::S_DIM_>>> riccatiEventPtrStock_;
  std::vector<std::shared_ptr<IntegratorBase<riccati_equations_t::S_DIM_>>> riccatiIntegratorPtrStock_;
  std::vector<std::shared_ptr<error_equation_t>> errorEquationPtrStock_;
  std::vector<std::shared_ptr<SystemEventHandler<STATE_DIM>>> errorEventPtrStock_;
  std::vector<std::shared_ptr<IntegratorBase<STATE_DIM>>> errorIntegratorPtrStock_;

  // functions for controller and lagrange multiplier
  std::vector<EigenLinearInterpolation<state_input_matrix_t>> BmFunc_;
  std::vector<EigenLinearInterpolation<input_state_matrix_t>> PmFunc_;
  std::vector<EigenLinearInterpolation<input_matrix_t>> RmInverseFunc_;
  std::vector<EigenLinearInterpolation<input_vector_t>> RvFunc_;
  std::vector<EigenLinearInterpolation<input_vector_t>> EvProjectedFunc_;
  std::vector<EigenLinearInterpolation<input_state_matrix_t>> CmProjectedFunc_;
  std::vector<EigenLinearInterpolation<input_matrix_t>> DmProjectedFunc_;

  // function for Riccati error equation
  std::vector<EigenLinearInterpolation<state_matrix_t>> SmFuncs_;

 private:
  /**
   * Solves a set of Riccati equations for the partition in the given index.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] partitionIndex: The requested partition index to solve Riccati equations.
   * @param [in] SmFinal: The final Sm for Riccati equation.
   * @param [in] SvFinal: The final Sv for Riccati equation.
   * @param [in] sFinal: The final s for Riccati equation.
   */
  void solveRiccatiEquationsWorker(size_t workerIndex, size_t partitionIndex, const state_matrix_t& SmFinal, const state_vector_t& SvFinal,
                                   const eigen_scalar_t& sFinal);

  /**
   * Type_1 constraints error correction compensation which solves a set of error Riccati equations for the partition in the given index.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] partitionIndex: The requested partition index to solve Riccati equations.
   * @param [in] SveFinal: The final Sve for the current Riccati equation.
   */
  void solveErrorRiccatiEquationWorker(size_t workerIndex, size_t partitionIndex, const state_vector_t& SveFinal);

  /**
   * Integrates the riccati equation and generates the value function at the times set in nominal Time Trajectory.
   *
   * @param riccatiIntegrator [in] : Riccati integrator object
   * @param riccatiEquation [in] : Riccati equation object
   * @param nominalTimeTrajectory [in] : time trajectory produced in the forward rollout.
   * @param nominalEventsPastTheEndIndices [in] : Indices into nominalTimeTrajectory to point to times right after event times
   * @param allSsFinal [in] : Final value of the value function.
   * @param SsNormalizedTime [out] : Time trajectory of the value function.
   * @param SsNormalizedEventsPastTheEndIndices [out] : Indices into SsNormalizedTime to point to times right after event times
   * @param allSsTrajectory [out] : Value function in vector format.
   */
  void integrateRiccatiEquationNominalTime(IntegratorBase<riccati_equations_t::S_DIM_>& riccatiIntegrator,
                                           riccati_equations_t& riccatiEquation, const scalar_array_t& nominalTimeTrajectory,
                                           const size_array_t& nominalEventsPastTheEndIndices, s_vector_t allSsFinal,
                                           scalar_array_t& SsNormalizedTime, size_array_t& SsNormalizedEventsPastTheEndIndices,
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
   * @param SsNormalizedEventsPastTheEndIndices [out] : Indices into SsNormalizedTime to point to times right after event times
   * @param allSsTrajectory [out] : Value function in vector format.
   */
  void integrateRiccatiEquationAdaptiveTime(IntegratorBase<riccati_equations_t::S_DIM_>& riccatiIntegrator,
                                            riccati_equations_t& riccatiEquation, const scalar_array_t& nominalTimeTrajectory,
                                            const size_array_t& nominalEventsPastTheEndIndices, s_vector_t allSsFinal,
                                            scalar_array_t& SsNormalizedTime, size_array_t& SsNormalizedEventsPastTheEndIndices,
                                            s_vector_array_t& allSsTrajectory);
};

}  // namespace ocs2

#include "implementation/SLQ_BASE.h"

#endif /* SLQ_BASE_OCS2_H_ */
