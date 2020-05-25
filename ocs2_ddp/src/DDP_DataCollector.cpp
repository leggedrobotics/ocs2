/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <ocs2_ddp/DDP_DataCollector.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
DDP_DataCollector::DDP_DataCollector(const RolloutBase* rolloutPtr, const DerivativesBase* systemDerivativesPtr,
                                     const ConstraintBase* systemConstraintsPtr, const CostFunctionBase* costFunctionPtr)

    : rolloutPtr_(rolloutPtr->clone()),
      systemDerivativesPtr_(systemDerivativesPtr->clone()),
      systemConstraintsPtr_(systemConstraintsPtr->clone()),
      costFunctionPtr_(costFunctionPtr->clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
void DDP_DataCollector::collect(const GaussNewtonDDP* constDdpPtr) {
  // TODO(mspieler): avoid const_cast
  auto* ddpPtr = const_cast<GaussNewtonDDP*>(constDdpPtr);

  /*
   * Data which should be copied
   */
  // initial time and state plus final time
  initTime_ = ddpPtr->initTime_;
  finalTime_ = ddpPtr->finalTime_;
  initState_ = ddpPtr->initState_;

  // active partitions range: [initActivePartition_, finalActivePartition_]
  initActivePartition_ = ddpPtr->initActivePartition_;
  finalActivePartition_ = ddpPtr->finalActivePartition_;

  // data resizing
  bool numPartitionsChanged = numPartitions_ != ddpPtr->numPartitions_;
  if (numPartitionsChanged) {
    resizeDataContainer(ddpPtr->numPartitions_, ddpPtr->stateDim_, ddpPtr->inputDim_);
  }

  numPartitions_ = ddpPtr->numPartitions_;
  partitioningTimes_ = ddpPtr->partitioningTimes_;

  rewindCounter_ = ddpPtr->rewindCounter_;

  modeSchedule_ = ddpPtr->getModeSchedule();

  // optimized controller
  optimizedControllersStock_ = ddpPtr->nominalControllersStock_;

  // nominal trajectories (LQ approximation is around the cached trajectories)
  nominalPostEventIndicesStock_ = ddpPtr->cachedPostEventIndicesStock_;
  nominalTimeTrajectoriesStock_ = ddpPtr->cachedTimeTrajectoriesStock_;
  nominalStateTrajectoriesStock_ = ddpPtr->cachedStateTrajectoriesStock_;
  nominalInputTrajectoriesStock_ = ddpPtr->cachedInputTrajectoriesStock_;

  /*
   * Data which can be swapped. Note that these variables should have correct size.
   * Otherwise use setOptimizer() to construct them with correct size
   */
  // model data trajectory
  modelDataTrajectoriesStock_.swap(ddpPtr->cachedModelDataTrajectoriesStock_);

  // event times model data
  modelDataEventTimesStock_.swap(ddpPtr->cachedModelDataEventTimesStock_);

  // projected model data trajectory
  projectedModelDataTrajectoriesStock_.swap(ddpPtr->cachedProjectedModelDataTrajectoriesStock_);

  // Riccati modification
  riccatiModificationTrajectoriesStock_.swap(ddpPtr->cachedRiccatiModificationTrajectoriesStock_);

  // terminal cost which is interpreted as the Heuristic function
  sHeuristics_ = ddpPtr->sHeuristics_;
  SvHeuristics_.swap(ddpPtr->SvHeuristics_);
  SmHeuristics_.swap(ddpPtr->SmHeuristics_);

  // Riccati coefficients
  SsTimeTrajectoriesStock_.swap(ddpPtr->SsTimeTrajectoryStock_);
  SsNormalizedTimeTrajectoriesStock_.swap(ddpPtr->SsNormalizedTimeTrajectoryStock_);
  SsNormalizedEventsPastTheEndIndecesStock_.swap(ddpPtr->SsNormalizedEventsPastTheEndIndecesStock_);
  SmTrajectoriesStock_.swap(ddpPtr->SmTrajectoryStock_);
  SvTrajectoriesStock_.swap(ddpPtr->SvTrajectoryStock_);
  sTrajectoriesStock_.swap(ddpPtr->sTrajectoryStock_);

  // state-input constraints derivatives w.r.t. to the event times
  calculateStateInputConstraintsSensitivity(constDdpPtr, nominalTimeTrajectoriesStock_, nominalStateTrajectoriesStock_,
                                            nominalInputTrajectoriesStock_, EvDevEventTimesTrajectoryStockSet_,
                                            EvDevEventTimesProjectedTrajectoryStockSet_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
void DDP_DataCollector::calculateStateInputConstraintsSensitivity(const GaussNewtonDDP* constDdpPtr,
                                                                  const std::vector<scalar_array_t>& timeTrajectoriesStock,
                                                                  const vector_array2_t& stateTrajectoriesStock,
                                                                  const vector_array2_t& inputTrajectoriesStock,
                                                                  vector_array3_t& EvDevEventTimesTrajectoryStockSet,
                                                                  vector_array3_t& EvDevEventTimesProjectedTrajectoriesStockSet) {
  auto* ddpPtr = const_cast<GaussNewtonDDP*>(constDdpPtr);

  const size_t numEventTimes = constDdpPtr->getModeSchedule().eventTimes.size();

  // resizing EvDev container
  EvDevEventTimesTrajectoryStockSet.resize(numEventTimes);
  for (auto& EvDevEventTimesTrajectoryStock : EvDevEventTimesTrajectoryStockSet) {
    EvDevEventTimesTrajectoryStock.resize(constDdpPtr->numPartitions_);
    for (size_t i = 0; i < constDdpPtr->numPartitions_; i++) {
      EvDevEventTimesTrajectoryStock[i].resize(timeTrajectoriesStock[i].size());
    }  // end of i loop
  }
  // resizing EvDevProjected container
  EvDevEventTimesProjectedTrajectoriesStockSet.resize(numEventTimes);
  for (auto& EvDevEventTimesProjectedTrajectoriesStock : EvDevEventTimesProjectedTrajectoriesStockSet) {
    EvDevEventTimesProjectedTrajectoriesStock.resize(constDdpPtr->numPartitions_);
    for (size_t i = 0; i < constDdpPtr->numPartitions_; i++) {
      EvDevEventTimesProjectedTrajectoriesStock[i].resize(timeTrajectoriesStock[i].size());
    }  // end of i loop
  }

  for (size_t i = 0; i < constDdpPtr->numPartitions_; i++) {
    for (size_t k = 0; k < timeTrajectoriesStock[i].size(); k++) {
      // set
      systemConstraintsPtr_->setCurrentStateAndControl(timeTrajectoriesStock[i][k], stateTrajectoriesStock[i][k],
                                                       inputTrajectoriesStock[i][k]);

      // evaluation
      vector_array_t g1DevArray(numEventTimes);
      const auto nc1 = systemConstraintsPtr_->numStateInputConstraint(timeTrajectoriesStock[i][k]);
      systemConstraintsPtr_->getConstraint1DerivativesEventTimes(g1DevArray);

      // if derivatives where available
      if (g1DevArray.size() > 0) {
        if (g1DevArray.size() != numEventTimes) {
          throw std::runtime_error("Incorrect array dimension for constraint1 derivatives w.r.t. event times.");
        }

        for (size_t j = 0; j < numEventTimes; j++) {
          EvDevEventTimesTrajectoryStockSet[j][i][k] = g1DevArray[j].head(nc1);
          EvDevEventTimesProjectedTrajectoriesStockSet[j][i][k] =
              riccatiModificationTrajectoriesStock_[i][k].constraintRangeProjector_ * EvDevEventTimesTrajectoryStockSet[j][i][k];
        }  // end of j loop

      } else {
        for (size_t j = 0; j < numEventTimes; j++) {
          EvDevEventTimesTrajectoryStockSet[j][i][k].setZero(nc1);
          EvDevEventTimesProjectedTrajectoriesStockSet[j][i][k].setZero();
        }  // end of j loop
      }
    }  // end of k loop
  }    // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
void DDP_DataCollector::resizeDataContainer(size_t numPartitions, size_t stateDim, size_t inputDim) {
  if (numPartitions == 0) {
    throw std::runtime_error("The number of Partitions cannot be zero!");
  }

  /*
   * Data which should be copied
   */
  // optimized controller
  optimizedControllersStock_.resize(numPartitions, LinearController(stateDim, inputDim));

  // nominal trajectories
  nominalTimeTrajectoriesStock_.resize(numPartitions);
  nominalPostEventIndicesStock_.resize(numPartitions);
  nominalStateTrajectoriesStock_.resize(numPartitions);
  nominalInputTrajectoriesStock_.resize(numPartitions);

  /*
   * Data which can be swapped. Note that these variables should have correct size.
   * Otherwise use setOptimizer() to construct them with correct size
   */
  // model data trajectory
  modelDataTrajectoriesStock_.resize(numPartitions);

  // event times model data
  modelDataEventTimesStock_.resize(numPartitions);

  // projected model data trajectory
  projectedModelDataTrajectoriesStock_.resize(numPartitions);

  // Riccati modification
  riccatiModificationTrajectoriesStock_.resize(numPartitions);

  // Riccati coefficients
  SsTimeTrajectoriesStock_.resize(numPartitions);
  SsNormalizedTimeTrajectoriesStock_.resize(numPartitions);
  SsNormalizedEventsPastTheEndIndecesStock_.resize(numPartitions);
  SmTrajectoriesStock_.resize(numPartitions);
  SvTrajectoriesStock_.resize(numPartitions);
  sTrajectoriesStock_.resize(numPartitions);
}

}  // namespace ocs2
