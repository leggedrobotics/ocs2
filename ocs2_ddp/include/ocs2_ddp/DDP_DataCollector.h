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

#include <ocs2_core/Types.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include "GaussNewtonDDP.h"

namespace ocs2 {

/**
 * Collects the required data from DDP instance. It uses swap method wherever it is possible.
 */
class DDP_DataCollector {
 public:
  /**
   * Default constructor.
   */
  DDP_DataCollector() = default;

  /**
   * Constructor.
   *
   * @param [in] rolloutPtr: The rollout class used for simulating the system dynamics.
   * @param [in] systemDynamicsPtr: The system dynamics and derivatives for the subsystems.
   * @param [in] systemConstraintsPtr: The system constraint function and its derivatives for subsystems.
   * @param [in] costFunctionPtr: The cost function (intermediate and final costs) and its derivatives for subsystems.
   */
  DDP_DataCollector(const RolloutBase* rolloutPtr, const SystemDynamicsBase* systemDynamicsPtr, const ConstraintBase* systemConstraintsPtr,
                    const CostFunctionBase* costFunctionPtr);

  /**
   * Default destructor.
   */
  ~DDP_DataCollector() = default;

  /**
   * Collects the required data from DDP instance. It uses swap method wherever it is possible.
   *
   * @param constDdpPtr: A pointer to the DDP instance.
   */
  void collect(const GaussNewtonDDP* constDdpPtr);

  /******************
   * DDP variables image
   ******************/
  scalar_t initTime_ = 0.0;
  scalar_t finalTime_ = 0.0;
  vector_t initState_;

  size_t initActivePartition_ = 0;
  size_t finalActivePartition_ = 0;
  size_t numPartitions_ = 0;
  scalar_array_t partitioningTimes_;

  unsigned long long int rewindCounter_ = 0;

  ModeSchedule modeSchedule_;

  std::vector<LinearController> optimizedControllersStock_;

  scalar_array2_t nominalTimeTrajectoriesStock_;
  size_array2_t nominalPostEventIndicesStock_;
  vector_array2_t nominalStateTrajectoriesStock_;
  vector_array2_t nominalInputTrajectoriesStock_;

  // model data trajectory
  std::vector<std::vector<ModelData>> modelDataTrajectoriesStock_;

  // event times model data
  std::vector<std::vector<ModelData>> modelDataEventTimesStock_;

  // projected model data trajectory
  std::vector<std::vector<ModelData>> projectedModelDataTrajectoriesStock_;

  // Riccati modification
  std::vector<std::vector<riccati_modification::Data>> riccatiModificationTrajectoriesStock_;

  // TODO(mspieler): delete this
  //  matrix_array2_t RmInverseTrajectoriesStock_;
  //  matrix_array2_t RmInvConstrainedCholTrajectoryStock_;
  //  matrix_array2_t DmDagerTrajectoriesStock_;

  // final cost which is interpreted as the Heuristic function
  ScalarFunctionQuadraticApproximation heuristics_;

  scalar_array2_t SsTimeTrajectoriesStock_;
  scalar_array2_t SsNormalizedTimeTrajectoriesStock_;
  size_array2_t SsNormalizedEventsPastTheEndIndecesStock_;
  scalar_array2_t sTrajectoriesStock_;
  vector_array2_t SvTrajectoriesStock_;
  matrix_array2_t SmTrajectoriesStock_;

  /******************
   * DDP missing variables
   ******************/
  vector_array3_t EvDevEventTimesTrajectoryStockSet_;           // state-input constraint derivative w.r.t. event times
  vector_array3_t EvDevEventTimesProjectedTrajectoryStockSet_;  // DmDager * EvDevEventTimes

 protected:
  /**
   * Resizes the data of the other-class's member.
   *
   * @param numPartitions: Number of partitions.
   */
  void resizeDataContainer(size_t numPartitions);

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
  void calculateStateInputConstraintsSensitivity(const GaussNewtonDDP* constDdpPtr,
                                                 const std::vector<scalar_array_t>& timeTrajectoriesStock,
                                                 const vector_array2_t& stateTrajectoriesStock,
                                                 const vector_array2_t& inputTrajectoriesStock,
                                                 vector_array3_t& EvDevEventTimesTrajectoriesStockSet,
                                                 vector_array3_t& EvDevEventTimesProjectedTrajectoriesStockSet);

 private:
  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::unique_ptr<SystemDynamicsBase> systemDynamicsPtr_;
  std::unique_ptr<ConstraintBase> systemConstraintsPtr_;
  std::unique_ptr<CostFunctionBase> costFunctionPtr_;
};

}  // namespace ocs2
