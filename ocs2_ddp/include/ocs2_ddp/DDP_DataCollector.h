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

#include "ocs2_ddp/GaussNewtonDDP.h"
#include "ocs2_oc/rollout/TimeTriggeredRollout.h"

namespace ocs2 {

/**
 * Collects the required data from DDP instance. It uses swap method wherever it is possible.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class DDP_DataCollector {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using ddp_t = GaussNewtonDDP<STATE_DIM, INPUT_DIM>;

  using Ptr = std::shared_ptr<DDP_DataCollector<STATE_DIM, INPUT_DIM>>;

  using controller_t = typename ddp_t::controller_t;
  using controller_ptr_array_t = typename ddp_t::controller_ptr_array_t;
  using linear_controller_t = typename ddp_t::linear_controller_t;
  using linear_controller_array_t = typename ddp_t::linear_controller_array_t;
  using size_array_t = typename ddp_t::size_array_t;
  using size_array2_t = typename ddp_t::size_array2_t;
  using scalar_t = typename ddp_t::scalar_t;
  using scalar_array_t = typename ddp_t::scalar_array_t;
  using scalar_array2_t = typename ddp_t::scalar_array2_t;
  using scalar_array3_t = typename ddp_t::scalar_array3_t;
  using state_vector_t = typename ddp_t::state_vector_t;
  using state_vector_array_t = typename ddp_t::state_vector_array_t;
  using state_vector_array2_t = typename ddp_t::state_vector_array2_t;
  using input_vector_array_t = typename ddp_t::input_vector_array_t;
  using input_vector_array2_t = typename ddp_t::input_vector_array2_t;
  using state_matrix_array_t = typename ddp_t::state_matrix_array_t;
  using state_matrix_t = typename ddp_t::state_matrix_t;
  using state_matrix_array2_t = typename ddp_t::state_matrix_array2_t;
  using dynamic_vector_t = typename ddp_t::dynamic_vector_t;
  using dynamic_vector_array_t = typename ddp_t::dynamic_vector_array_t;
  using dynamic_vector_array2_t = typename ddp_t::dynamic_vector_array2_t;
  using dynamic_vector_array3_t = typename ddp_t::dynamic_vector_array3_t;
  using dynamic_matrix_t = typename ddp_t::dynamic_matrix_t;
  using dynamic_matrix_array_t = typename ddp_t::dynamic_matrix_array_t;
  using dynamic_matrix_array2_t = typename ddp_t::dynamic_matrix_array2_t;
  using dynamic_matrix_array3_t = typename ddp_t::dynamic_matrix_array3_t;

  using derivatives_base_t = typename ddp_t::derivatives_base_t;
  using constraint_base_t = typename ddp_t::constraint_base_t;
  using cost_function_base_t = typename ddp_t::cost_function_base_t;
  using rollout_base_t = typename ddp_t::rollout_base_t;

  /**
   * Default constructor.
   */
  DDP_DataCollector() = default;

  /**
   * Constructor.
   *
   * @param [in] rolloutPtr: The rollout class used for simulating the system dynamics.
   * @param [in] systemDerivativesPtr: The system dynamics derivatives for subsystems of the system.
   * @param [in] systemConstraintsPtr: The system constraint function and its derivatives for subsystems.
   * @param [in] costFunctionPtr: The cost function (intermediate and terminal costs) and its derivatives for subsystems.
   */
  DDP_DataCollector(const rollout_base_t* rolloutPtr, const derivatives_base_t* systemDerivativesPtr,
                    const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr);

  /**
   * Default destructor.
   */
  ~DDP_DataCollector() = default;

  /**
   * Collects the required data from DDP instance. It uses swap method wherever it is possible.
   *
   * @param constDdpPtr: A pointer to the DDP instance.
   */
  void collect(const ddp_t* constDdpPtr);

  /******************
   * DDP variables image
   ******************/
  scalar_t initTime_;
  scalar_t finalTime_;
  state_vector_t initState_;

  size_t initActivePartition_;
  size_t finalActivePartition_;
  size_t numPartitions_ = 0;
  scalar_array_t partitioningTimes_;

  unsigned long long int rewindCounter_;

  scalar_array_t eventTimes_;
  size_array_t subsystemsSequence_;

  linear_controller_array_t optimizedControllersStock_;

  scalar_array2_t nominalTimeTrajectoriesStock_;
  size_array2_t nominalPostEventIndicesStock_;
  state_vector_array2_t nominalStateTrajectoriesStock_;
  input_vector_array2_t nominalInputTrajectoriesStock_;

  // model data trajectory
  ModelDataBase::array2_t modelDataTrajectoriesStock_;

  // event times model data
  ModelDataBase::array2_t modelDataEventTimesStock_;

  // projected model data trajectory
  ModelDataBase::array2_t projectedModelDataTrajectoriesStock_;

  // Riccati modification
  RiccatiModificationBase::array2_t riccatiModificationTrajectoriesStock_;

  // TODO: delete this
  //  dynamic_matrix_array2_t RmInverseTrajectoriesStock_;
  //  dynamic_matrix_array2_t RmInvConstrainedCholTrajectoryStock_;
  //  dynamic_matrix_array2_t DmDagerTrajectoriesStock_;

  // terminal cost which is interpreted as the Heuristic function
  scalar_t sHeuristics_;
  dynamic_vector_t SvHeuristics_;
  dynamic_matrix_t SmHeuristics_;

  scalar_array2_t SsTimeTrajectoriesStock_;
  scalar_array2_t SsNormalizedTimeTrajectoriesStock_;
  size_array2_t SsNormalizedEventsPastTheEndIndecesStock_;
  scalar_array2_t sTrajectoriesStock_;
  dynamic_vector_array2_t SvTrajectoriesStock_;
  dynamic_matrix_array2_t SmTrajectoriesStock_;

  /******************
   * DDP missing variables
   ******************/
  dynamic_vector_array3_t EvDevEventTimesTrajectoryStockSet_;           // state-input constraint derivative w.r.t. event times
  dynamic_vector_array3_t EvDevEventTimesProjectedTrajectoryStockSet_;  // DmDager * EvDevEventTimes

 protected:
  /**
   * Resizes the data of the other-class's member.
   *
   * @param numPartitions: Number of partitions.
   * @param otherPtr: A pointer to the OTHER_CLASS instance
   * @return True if number of partitions is changed.
   */
  void resizeDataContainer(const size_t& numPartitions);

  /**
   * Calculates sensitivity of the state-input constraints to event times.
   *
   * @param [in] constDdpPtr: A pointer to the DDP instance.
   * @param [in] timeTrajectoriesStock: The time trajectory stamp.
   * @param [in] stateTrajectoriesStock: The state trajectory.
   * @param [in] inputTrajectoriesStock: The control input trajectory.
   * @param [out] EvDevEventTimesTrajectoriesStockSet: The sensitivity of the state-input constraints to the event times. Here
   * EvDevEventTimesTrajectoriesStockSet[i] is the EvDevEventTimesTrajectoriesStock for the i'th event time.
   * @param [out] EvDevEventTimesProjectedTrajectoriesStockSet: The projected sensitivity of the state-input constraints to the event times,
   * Defined as (DmDager * EvDevEventTimes).
   */
  void calculateStateInputConstraintsSensitivity(const ddp_t* constDdpPtr, const std::vector<scalar_array_t>& timeTrajectoriesStock,
                                                 const state_vector_array2_t& stateTrajectoriesStock,
                                                 const input_vector_array2_t& inputTrajectoriesStock,
                                                 dynamic_vector_array3_t& EvDevEventTimesTrajectoriesStockSet,
                                                 dynamic_vector_array3_t& EvDevEventTimesProjectedTrajectoriesStockSet);

 private:
  std::unique_ptr<rollout_base_t> rolloutPtr_;
  std::unique_ptr<derivatives_base_t> systemDerivativesPtr_;
  std::unique_ptr<constraint_base_t> systemConstraintsPtr_;
  std::unique_ptr<cost_function_base_t> costFunctionPtr_;
};

}  // namespace ocs2

#include "implementation/DDP_DataCollector.h"
