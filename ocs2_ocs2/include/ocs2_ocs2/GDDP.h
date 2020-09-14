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

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <numeric>
#include <type_traits>
#include <vector>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/integration/Integrator.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/Lookup.h>

#include <ocs2_ddp/DDP_DataCollector.h>

#include "ocs2_ocs2/GDDP_Settings.h"
#include "ocs2_ocs2/sensitivity_equations/BvpSensitivityEquations.h"
#include "ocs2_ocs2/sensitivity_equations/BvpSensitivityErrorEquations.h"
#include "ocs2_ocs2/sensitivity_equations/RolloutSensitivityEquations.h"
#include "ocs2_ocs2/sensitivity_equations/SensitivitySequentialRiccatiEquations.h"

namespace ocs2 {

/**
 * GDDP class for computing gradient of the cost function w.r.t. event times.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class GDDP {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using ddp_data_collector_t = DDP_DataCollector<STATE_DIM, INPUT_DIM>;

  using bvp_sensitivity_equations_t = BvpSensitivityEquations<STATE_DIM, INPUT_DIM>;
  using bvp_sensitivity_error_equations_t = BvpSensitivityErrorEquations<STATE_DIM, INPUT_DIM>;
  using rollout_sensitivity_equations_t = RolloutSensitivityEquations<STATE_DIM, INPUT_DIM>;
  using riccati_sensitivity_equations_t = SensitivitySequentialRiccatiEquations<STATE_DIM, INPUT_DIM>;

  using linear_controller_t = LinearController<STATE_DIM, INPUT_DIM>;
  using linear_controller_array_t = typename linear_controller_t::array_t;

  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using size_array_t = typename DIMENSIONS::size_array_t;
  using size_array2_t = typename DIMENSIONS::size_array2_t;
  using scalar_array3_t = typename DIMENSIONS::scalar_array3_t;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using scalar_array2_t = typename DIMENSIONS::scalar_array2_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using state_vector_array2_t = typename DIMENSIONS::state_vector_array2_t;
  using state_vector_array3_t = typename DIMENSIONS::state_vector_array3_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
  using input_vector_array2_t = typename DIMENSIONS::input_vector_array2_t;
  using input_vector_array3_t = typename DIMENSIONS::input_vector_array3_t;
  using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t;
  using input_state_matrix_array_t = typename DIMENSIONS::input_state_matrix_array_t;
  using input_state_matrix_array2_t = typename DIMENSIONS::input_state_matrix_array2_t;
  using state_matrix_t = typename DIMENSIONS::state_matrix_t;
  using state_matrix_array_t = typename DIMENSIONS::state_matrix_array_t;
  using state_matrix_array2_t = typename DIMENSIONS::state_matrix_array2_t;
  using state_matrix_array3_t = typename DIMENSIONS::state_matrix_array3_t;
  using input_matrix_t = typename DIMENSIONS::input_matrix_t;
  using input_matrix_array_t = typename DIMENSIONS::input_matrix_array_t;
  using input_matrix_array2_t = typename DIMENSIONS::input_matrix_array2_t;
  using input_matrix_array3_t = typename DIMENSIONS::input_matrix_array3_t;
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
  using dynamic_vector_t = typename DIMENSIONS::dynamic_vector_t;
  using dynamic_vector_array_t = typename DIMENSIONS::dynamic_vector_array_t;
  using dynamic_vector_array2_t = typename DIMENSIONS::dynamic_vector_array2_t;
  using dynamic_matrix_t = typename DIMENSIONS::dynamic_matrix_t;
  using dynamic_input_matrix_t = typename DIMENSIONS::dynamic_input_matrix_t;

  /**
   * Constructor.
   *
   * @param [in] settings: Structure containing the settings for the GDDP algorithm.
   */
  explicit GDDP(GDDP_Settings gddpSettings);

  /**
   * Default destructor.
   */
  virtual ~GDDP() = default;

  /**
   * Gets the calculated rollout's sensitivity to an event time.
   *
   * @param [in] eventTimeIndex: Event time index.
   * @param [out] sensitivityTimeTrajectoriesStock: time stamps of the sensitivity values.
   * @param [out] sensitivityStateTrajectoriesStock: state trajectory sensitivity to the switching times.
   * @param [out] sensitivityInputTrajectoriesStock: control input trajectory sensitivity to the switching times.
   */
  void getRolloutSensitivity2EventTime(const size_t& eventTimeIndex, std::vector<scalar_array_t>& sensitivityTimeTrajectoriesStock,
                                       state_matrix_array2_t& sensitivityStateTrajectoriesStock,
                                       input_matrix_array2_t& sensitivityInputTrajectoriesStock);

  /**
   * Gets a reference to the Options structure.
   *
   * @return a reference to the Options structure.
   */
  GDDP_Settings& settings();

  /**
   * Calculates the cost function's derivatives w.r.t. event times.
   *
   * @param [out] costFunctionDerivative: cost function's derivatives w.r.t. event times.
   */
  template <typename Derived>
  void getCostFuntionDerivative(Eigen::MatrixBase<Derived> const& costFunctionDerivative) const;

  /**
   * Gets a constant reference to the event time vector.
   *
   * @return A constant reference to the event time vector.
   */
  const scalar_array_t& eventTimes() const;

  /**
   * Runs the GSLQ to compute the gradient of the cost function w.r.t. the event times.
   *
   * @param [in] dcPtr: A constant pointer to SLQ data collector which already collected the SLQ variables.
   */
  void run(const ddp_data_collector_t* dataCollectorPtr);

 protected:
  /**
   * Runs the LQ-based algorithm to compute the value function derivatives wr.t. event times.
   */
  void runLQBasedMethod();

  /**
   * Runs the Sweeping-BVP algorithm to compute the cost function derivatives wr.t. event times.
   */
  void runSweepingBVPMethod();

  /**
   * Sets up optimizer for different number of partitions.
   *
   * @param [in] numPartitions: number of partitions.
   */
  virtual void setupOptimizer(const size_t& numPartitions);

  /**
   * Computes the costate over the given rollout.
   *
   * @param [in] timeTrajectoriesStock: the inquiry rollout time
   * @param [in] stateTrajectoriesStock: the inquiry rollout state
   * @param [out] costateTrajectoriesStock: costate vector for the given trajectory
   * @param [in] learningRate: the learning rate.
   */
  void calculateRolloutCostate(const scalar_array2_t& timeTrajectoriesStock, const state_vector_array2_t& stateTrajectoriesStock,
                               state_vector_array2_t& costateTrajectoriesStock, scalar_t learningRate = 0.0);

  /**
   * Computes the nominal costate over the given time.
   *
   * @param [in] timeTrajectoriesStock: the inquiry rollout time
   * @param [out] costateTrajectoriesStock: costate vector for the given trajectory
   * @param [in] learningRate: the learning rate.
   */
  void calculateRolloutCostate(const std::vector<scalar_array_t>& timeTrajectoriesStock, state_vector_array2_t& costateTrajectoriesStock);

  /**
   * Computes the Lagrange multiplier of the state-input constraint over the given time trajectory.
   *
   * @param [in] timeTrajectoriesStock: the inquiry rollout time
   * @param [out] lagrangeTrajectoriesStock: lagrangeMultiplier value over the given trajectory
   */
  void calculateNominalRolloutLagrangeMultiplier(const scalar_array2_t& timeTrajectoriesStock,
                                                 dynamic_vector_array2_t& lagrangeTrajectoriesStock);

  /**
   * Computes the equivalent system formulation multiplier. which is
   * \f$ \frac{\delta_{i,j}-\delta_{i-1,j}}{t_{i}-t_{i-1}} \f$
   * where i is activeSubsystem and j is eventTimeIndex.
   *
   * @param [in] eventTimeIndex: Event time index.
   * @param [in] activeSubsystem: Current active subsystem index.
   * @param [out] multiplier: Equivalent system formulation multiplier.
   */
  void computeEquivalentSystemMultiplier(const size_t& eventTimeIndex, const size_t& activeSubsystem, scalar_t& multiplier) const;

  /**
   * Integrates the sensitivity equation of the rollout w.r.t. event times.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] eventTimeIndex: Event time index.
   * @param [in] controllersStock: Nominal controller.
   * @param [in] LvTrajectoriesStock: Controller's feedforward sensitivity
   * @param [in] sensitivityTimeTrajectoriesStock: Integration time trajectory.
   * @param [in] postEventIndicesStock: Post event indices array.
   * @param [out] sensitivityStateTrajectoriesStock: Array of state sensitivity trajectory.
   * @param [out] sensitivityInputTrajectoriesStock: Array of input sensitivity trajectory.
   */
  void propagateRolloutSensitivity(size_t workerIndex, const size_t& eventTimeIndex, const linear_controller_array_t& controllersStock,
                                   const input_vector_array2_t& LvTrajectoriesStock,
                                   const scalar_array2_t& sensitivityTimeTrajectoriesStock, const size_array2_t& postEventIndicesStock,
                                   state_vector_array2_t& sensitivityStateTrajectoriesStock,
                                   input_vector_array2_t& sensitivityInputTrajectoriesStock);

  /**
   * Approximates nominal LQ problem sensitivity to event times.
   *
   * @param [in] sensitivityStateTrajectoriesStock: Array of state sensitivity trajectory.
   * @param [in] sensitivityInputTrajectoriesStock: Array of input sensitivity trajectory.
   * @param [out] nablaqTrajectoriesStock: Sensitivity of the cost's zero order variation.
   * @param [out] nablaQvTrajectoriesStock: Sensitivity of the cost's first order state variation.
   * @param [out] nablaRvTrajectoriesStock: Sensitivity of the cost's first order input variation.
   * @param [out] nablaqFinalStock: Sensitivity of the final cost's zero order variation.
   * @param [out] nablaQvFinalStock: Sensitivity of the final cost's first order state variation.
   */
  void approximateNominalLQPSensitivity2EventTime(const state_vector_array2_t& sensitivityStateTrajectoriesStock,
                                                  const input_vector_array2_t& sensitivityInputTrajectoriesStock,
                                                  scalar_array2_t& nablaqTrajectoriesStock, state_vector_array2_t& nablaQvTrajectoriesStock,
                                                  input_vector_array2_t& nablaRvTrajectoriesStock, scalar_array2_t& nablaqFinalStock,
                                                  state_vector_array2_t& nablaQvFinalStock) const;

  /**
   * Approximates nominal LQ problem sensitivity to event times.
   *
   * @param [in] sensitivityFinalState: Final state sensitivity.
   * @param [out] nablasHeuristics: Sensitivity of the Heuristics zero order variation.
   * @param [out] nablaSvHeuristics: Sensitivity of the Heuristics first order state variation.
   */
  void approximateNominalHeuristicsSensitivity2EventTime(const state_vector_t& sensitivityFinalState, scalar_t& nablasHeuristics,
                                                         state_vector_t& nablaSvHeuristics) const;

  /**
   * Solves the SLQ Riccati equations sensitivity differential equations.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] eventTimeIndex: Event time index.
   * @param [in] learningRate: learning rate typically should be zero.
   * @param [in] nablasHeuristics: Sensitivity of the Heuristics zero order variation.
   * @param [in] nablaSvHeuristics: Sensitivity of the Heuristics first order state variation.
   * @param [in] nablaSmHeuristics: Sensitivity of the Heuristics second order state variation.
   * @param [out] nablasTrajectoriesStock: Sensitivity of the Riccati equations's zero order variation.
   * @param [out] nablaSvTrajectoriesStock: Sensitivity of the Riccati equations's first order variation.
   * @param [out] nablaSmTrajectoriesStock: Sensitivity of the Riccati equations's second order variation.
   */
  void solveSensitivityRiccatiEquations(size_t workerIndex, const size_t& eventTimeIndex, const scalar_t& learningRate,
                                        const scalar_t& nablasHeuristics, const state_vector_t& nablaSvHeuristics,
                                        const state_matrix_t& nablaSmHeuristics, scalar_array2_t& nablasTrajectoriesStock,
                                        state_vector_array2_t& nablaSvTrajectoriesStock, state_matrix_array2_t& nablaSmTrajectoriesStock);

  /**
   * Solves a boundary value problem using a Riccati approach which later be used to
   * compute controller's feedforward sensitivity w.r.t. event times.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] eventTimeIndex: Event time index.
   * @param [in] MvFinal: The final Heuristic value for Mv.
   * @param [in] MveFinal: The final Heuristic value for Mve.
   * @param [out] MvTrajectoriesStock: Boundary value problem solution for Mv.
   * @param [out] MvTrajectoriesStock: Boundary value problem solution for Mve.
   */
  void solveSensitivityBVP(size_t workerIndex, const size_t& eventTimeIndex, const state_vector_t& MvFinal, const state_vector_t& MveFinal,
                           state_vector_array2_t& MvTrajectoriesStock, state_vector_array2_t& MveTrajectoriesStock);

  /**
   * Calculates controller's feedforward part sensitivity for the LQ method.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] eventTimeIndex: Event time index.
   * @param [in] timeTrajectoriesStock: Time stamp of the Riccati solution
   * @param [in] nablaSvTrajectoriesStock: Sensitivity of the Riccati equations's first order variation.
   * @param [out] nablaLvTrajectoriesStock: Sensitivity of the control input increment to event times.
   */
  void calculateLQSensitivityControllerForward(size_t workerIndex, const size_t& eventTimeIndex,
                                               const scalar_array2_t& timeTrajectoriesStock,
                                               const state_vector_array2_t& nablaSvTrajectoriesStock,
                                               input_vector_array2_t& nablaLvTrajectoriesStock);

  /**
   * calculate the sensitivity of the control input increment to event times based on the BVP method.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] eventTimeIndex: Event time index.
   * @param [in] timeTrajectoriesStock: Time stamp of the BVP solution
   * @param [in] MvTrajectoriesStock: BVP solution for Mv.
   * @param [in] MveTrajectoriesStock: BVP solution for Mve.
   * @param [out] LvTrajectoriesStock: Sensitivity of the control input increment to event times.
   */
  void calculateBVPSensitivityControllerForward(size_t workerIndex, const size_t& eventTimeIndex,
                                                const scalar_array2_t& timeTrajectoriesStock,
                                                const state_vector_array2_t& MvTrajectoriesStock,
                                                const state_vector_array2_t& MveTrajectoriesStock,
                                                input_vector_array2_t& LvTrajectoriesStock);

  /**
   * Calculates the value function's derivative w.r.t. an event time for a given time and state.
   *
   * @param [in] eventTimeIndex: Event time index.
   * @param [in] time: The inquired time.
   * @param [in] state: The inquired state.
   * @param [out] valueFunctionDerivative: The value function's derivative w.r.t. an event time.
   */
  void getValueFuntionSensitivity(const size_t& eventTimeIndex, const scalar_t& time, const state_vector_t& state,
                                  scalar_t& valueFunctionDerivative);

  /**
   * calculates cost function derivative based on BVP solution
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] eventTimeIndex: Event time index.
   * @param [in] sensitivityStateTrajectoriesStock: Array of state sensitivity trajectory.
   * @param [in] sensitivityInputTrajectoriesStock: Array of input sensitivity trajectory.
   * @param [out] costDerivative: The cost function's derivative w.r.t. an event time.
   */
  void calculateCostDerivative(size_t workerIndex, const size_t& eventTimeIndex,
                               const state_vector_array2_t& sensitivityStateTrajectoriesStock,
                               const input_vector_array2_t& sensitivityInputTrajectoriesStock, scalar_t& costDerivative) const;

  /***********
   * Variables
   **********/
  GDDP_Settings gddpSettings_;

  size_t numPartitions_ = 0;
  size_t numEventTimes_ = 0;

  size_t activeEventTimeBeginIndex_;
  size_t activeEventTimeEndIndex_;

  scalar_array_t eventTimes_;

  /******************
   * SLQ data collector
   ******************/
  const ddp_data_collector_t* dataCollectorPtr_;

  /******************
   * SLQ missing variables
   ******************/
  state_vector_array2_t nominalCostateTrajectoriesStock_;
  dynamic_vector_array2_t nominalLagrangianTrajectoriesStock_;

  std::vector<std::shared_ptr<bvp_sensitivity_equations_t>> bvpSensitivityEquationsPtrStock_;
  std::vector<std::unique_ptr<IntegratorBase<STATE_DIM>>> bvpSensitivityIntegratorsPtrStock_;
  std::vector<std::shared_ptr<bvp_sensitivity_error_equations_t>> bvpSensitivityErrorEquationsPtrStock_;
  std::vector<std::unique_ptr<IntegratorBase<STATE_DIM>>> bvpSensitivityErrorIntegratorsPtrStock_;
  std::vector<std::shared_ptr<rollout_sensitivity_equations_t>> rolloutSensitivityEquationsPtrStock_;
  std::vector<std::unique_ptr<IntegratorBase<STATE_DIM>>> rolloutSensitivityIntegratorsPtrStock_;
  std::vector<std::shared_ptr<riccati_sensitivity_equations_t>> riccatiSensitivityEquationsPtrStock_;
  std::vector<std::unique_ptr<IntegratorBase<riccati_sensitivity_equations_t::S_DIM_>>> riccatiSensitivityIntegratorsPtrStock_;
  //
  scalar_array3_t nablaqTrajectoriesStockSet_;
  state_vector_array3_t nablaQvTrajectoriesStockSet_;
  input_vector_array3_t nablaRvTrajectoriesStockSet_;
  scalar_array3_t nablaqFinalStockSet_;
  state_vector_array3_t nablaQvFinalStockSet_;
  scalar_array_t nablasHeuristics_;
  state_vector_array_t nablaSvHeuristics_;

  scalar_array3_t nablasTrajectoriesStockSet_;
  state_vector_array3_t nablaSvTrajectoriesStockSet_;
  state_matrix_array3_t nablaSmTrajectoriesStockSet_;
  input_vector_array3_t nablaLvTrajectoriesStockSet_;

  state_vector_array3_t MvTrajectoriesStockSet_;   // Riccati solution for sensitivity controller feedforward
  state_vector_array3_t MveTrajectoriesStockSet_;  // Riccati solution for sensitivity controller feedforward error-correction term.
  input_vector_array3_t LvTrajectoriesStockSet_;   // sensitivity controller feedforward

  state_vector_array3_t sensitivityStateTrajectoriesStockSet_;
  input_vector_array3_t sensitivityInputTrajectoriesStockSet_;

  dynamic_vector_t nominalCostFuntionDerivative_;
};

}  // namespace ocs2

#include "implementation/GDDP.h"
