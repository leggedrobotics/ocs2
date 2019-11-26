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

#include <algorithm>
#include <array>
#include <iterator>
#include <memory>

#include <ocs2_frank_wolfe/GradientDescent.h>

#include <ocs2_ddp/SLQ.h>

#include "ocs2_ocs2/GDDP.h"

namespace ocs2 {

/**
 * OCS2Projected
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class OCS2Projected : private nlp::GradientDescent<double> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef nlp::GradientDescent<double> BASE;

  typedef SLQ<STATE_DIM, INPUT_DIM> slq_t;
  typedef GDDP<STATE_DIM, INPUT_DIM> gddp_t;
  typedef typename slq_t::Ptr slq_ptr_t;

  typedef SLQ_DataCollector<STATE_DIM, INPUT_DIM> slq_data_collector_t;
  typedef typename slq_data_collector_t::Ptr slq_data_collector_ptr_t;

  typedef typename slq_t::controlled_system_base_t controlled_system_base_t;
  typedef typename slq_t::derivatives_base_t derivatives_base_t;
  typedef typename slq_t::constraint_base_t constraint_base_t;
  typedef typename slq_t::cost_function_base_t cost_function_base_t;
  typedef typename slq_t::cost_desired_trajectories_t cost_desired_trajectories_t;
  typedef typename slq_t::operating_trajectories_base_t operating_trajectories_base_t;

  typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;

  typedef typename DIMENSIONS::controller_t controller_t;
  typedef typename DIMENSIONS::controller_array_t controller_array_t;
  typedef typename DIMENSIONS::controller_array2_t controller_array2_t;
  typedef typename DIMENSIONS::scalar_t scalar_t;
  typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
  typedef typename DIMENSIONS::eigen_scalar_t eigen_scalar_t;
  typedef typename DIMENSIONS::eigen_scalar_array_t eigen_scalar_array_t;
  typedef typename DIMENSIONS::state_vector_t state_vector_t;
  typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
  typedef typename DIMENSIONS::state_vector_array2_t state_vector_array2_t;
  typedef typename DIMENSIONS::input_vector_t input_vector_t;
  typedef typename DIMENSIONS::input_vector_array_t input_vector_array_t;
  typedef typename DIMENSIONS::input_vector_array2_t input_vector_array2_t;
  typedef typename DIMENSIONS::dynamic_vector_t dynamic_vector_t;
  typedef typename DIMENSIONS::dynamic_matrix_t dynamic_matrix_t;

  /**
   * Default constructor.
   */
  OCS2Projected() = default;

  /**
   * Constructor
   * @param [in] subsystemDynamicsPtr
   * @param [in] subsystemDerivativesPtr
   * @param [in] subsystemCostFunctionsPtr
   * @param [in] slqSettings
   * @param [in] stateOperatingPoints
   * @param [in] inputOperatingPoints
   */
  OCS2Projected(const controlled_system_base_t* systemDynamicsPtr, const derivatives_base_t* systemDerivativesPtr,
                const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr,
                const operating_trajectories_base_t* operatingTrajectoriesPtr, const SLQ_Settings& slqSettings = SLQ_Settings(),
                const GDDP_Settings& gddpSettings = GDDP_Settings(), std::shared_ptr<HybridLogicRules> logicRulesPtr = nullptr,
                const cost_function_base_t* heuristicsFunctionPtr = nullptr);

  /**
   * Default destructor.
   */
  virtual ~OCS2Projected() = default;

  /**
   * Gets cost function
   * @param [out] costFunction
   */
  void getCostFunction(scalar_t& costFunction) const;

  /**
   * Gets the function derivatives
   * @param [out] costFuntionDerivative
   */
  void getCostFunctionDerivative(dynamic_vector_t& costFuntionDerivative) const;

  /**
   * Gets the controller stocks
   * @param [out] controllersStock
   */
  void getController(controller_array_t& controllersStock) const;

  /**
   * Gets event times
   *
   * @param [out] eventTimes
   */
  void getEventTimes(scalar_array_t& eventTimes) const;

  /**
   * Gets trajectories
   * @param [out] nominalTimeTrajectoriesStock
   * @param [out] nominalStateTrajectoriesStock
   * @param [out] nominalInputTrajectoriesStock
   */
  void getNominalTrajectories(std::vector<scalar_array_t>& nominalTimeTrajectoriesStock,
                              state_vector_array2_t& nominalStateTrajectoriesStock,
                              input_vector_array2_t& nominalInputTrajectoriesStock) const;

  /**
   * Gets iterations Log of SLQ.
   *
   * @param [out] iterationCost: Each SLQ iteration's cost.
   * @param [out] iterationISE1: Each SLQ iteration's type-1 constraints ISE.
   * @param [out] iterationISE2: Each SLQ iteration's type-2 constraints ISE.
   */
  void getSLQIterationsLog(eigen_scalar_array_t& slqIterationCost, eigen_scalar_array_t& slqIterationISE1,
                           eigen_scalar_array_t& slqIterationISE2) const;

  /**
   * Gets iterations log of OCS2.
   *
   * @param [out] iterationCost: Each OCS2 iteration's cost.
   */
  void getOCS2IterationsLog(eigen_scalar_array_t& iterationCost) const;

  /**
   * Return a reference to SLQ settings which can be used to modify the settings.
   *
   * @return a reference to SLQ settings.
   */
  SLQ_Settings& slqSettings();

  /**
   * Main run function
   *
   * @param initTime
   * @param initState
   * @param finalTime
   * @param initEventTimes
   * @param costDesiredTrajectories
   */
  void run(const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime, const scalar_array_t& partitioningTimes,
           const scalar_array_t& initEventTimes,
           const cost_desired_trajectories_t& costDesiredTrajectories = cost_desired_trajectories_t());

 private:
  /**
   * Finds nearest neighbor
   * @param [in] enquiryParameter
   * @return size_t:
   */
  size_t findNearestController(const dynamic_vector_t& enquiryParameter) const;

  /**
   * Calculates the coefficients of the linear equality constraints. \n
   * \f$ A_m \theta + B_v = 0\f$
   *
   * @param [out] Am: The \f$ A_m\f$ matrix.
   * @param [out] Bv: THe \f$ B_v \f$ vector.
   */
  void calculateLinearEqualityConstraint(dynamic_matrix_t& Am, dynamic_vector_t& Bv) override;

  /**
   * Calculates the gradient direction.
   *
   * @param [in] id: Solver ID
   * @param [in] parameters: The current parameter vector.
   * @param [in] gradient: The gradient at the current parameter vector.
   * @return boolean: The success flag.
   */
  virtual bool calculateGradient(const size_t& id, const dynamic_vector_t& parameters, dynamic_vector_t& gradient) override;

  /**
   * Calculates the cost function.
   *
   * @param [in] id: Solver ID
   * @param [in] parameters: The current parameter vector.
   * @param [in] cost: The cost for the current parameter vector.
   * @return boolean: The success flag.
   */
  virtual bool calculateCost(const size_t& id, const dynamic_vector_t& parameters, scalar_t& cost) override;

  /**
   * Get solution
   * @param [out] idStar
   */
  virtual void getSolution(size_t idStar) override;

  /**
   * rewind optimizer
   * @param [in] firstIndex
   * @param [in] initRun
   */
  void rewindOptimizer(const size_t& firstIndex, bool initRun = false);

  /**
   * Saves to bag
   * @param [in] id
   * @param [in] parameters
   */
  void saveToBag(size_t id, const dynamic_vector_t& parameters);

  /**
   * Sets up optimizer for different number of partitions.
   *
   * @param [in] numPartitions: number of partitions.
   */
  virtual void setupOptimizer(const size_t& numPartitions);

 private:
  SLQ_Settings slqSettings_;

  scalar_t initTime_;
  state_vector_t initState_;
  scalar_t finalTime_;
  size_t numSubsystems_ = 0;
  scalar_array_t partitioningTimes_;
  size_t numPartitions_;
  scalar_array_t initEventTimes_;

  // optimized solution variables
  scalar_t optimizedTotalCost_;
  scalar_t optimizedConstraint1ISE_;
  scalar_t optimizedConstraint2ISE_;
  scalar_array_t optimizedEventTimes_;
  controller_array_t optimizedControllersStock_;
  std::vector<scalar_array_t> optimizedTimeTrajectoriesStock_;
  state_vector_array2_t optimizedStateTrajectoriesStock_;
  input_vector_array2_t optimizedInputTrajectoriesStock_;
  dynamic_vector_t costFuntionDerivative_;

  scalar_t currentTotalCost_;
  dynamic_vector_t currentCostFuntionDerivative_;

  std::vector<dynamic_vector_t> parameterBag_;
  controller_array2_t controllersStockBag_;

  std::unique_ptr<gddp_t> gddpSolverPtr_;
  slq_data_collector_ptr_t slqDataCollectorPtr_;
  std::vector<slq_ptr_t> slqSolverPtrs_;

  eigen_scalar_array_t slqIterationCost_;
  eigen_scalar_array_t slqIterationISE1_;
  eigen_scalar_array_t slqIterationISE2_;
};

}  // namespace ocs2

#include "implementation/OCS2Projected.h"
