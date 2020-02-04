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

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
SLQ_DataCollector<STATE_DIM, INPUT_DIM>::SLQ_DataCollector(const rollout_base_t* rolloutPtr, const derivatives_base_t* systemDerivativesPtr,
                                                           const constraint_base_t* systemConstraintsPtr,
                                                           const cost_function_base_t* costFunctionPtr)

    : rolloutPtr_(rolloutPtr->clone()),
      systemDerivativesPtr_(systemDerivativesPtr->clone()),
      systemConstraintsPtr_(systemConstraintsPtr->clone()),
      costFunctionPtr_(costFunctionPtr->clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_DataCollector<STATE_DIM, INPUT_DIM>::collect(const slq_t* constSlqPtr) {
  auto* slqPtr = const_cast<slq_t*>(constSlqPtr);

  /*
   * Data which should be copied
   */
  // initial time and state plus final time
  initTime_ = slqPtr->initTime_;
  finalTime_ = slqPtr->finalTime_;
  initState_ = slqPtr->initState_;

  // active partitions range: [initActivePartition_, finalActivePartition_]
  initActivePartition_ = slqPtr->initActivePartition_;
  finalActivePartition_ = slqPtr->finalActivePartition_;

  // data resizing
  bool numPartitionsChanged = numPartitions_ != slqPtr->numPartitions_;
  if (numPartitionsChanged) {
    resizeDataContainer(slqPtr->numPartitions_);
  }

  numPartitions_ = slqPtr->numPartitions_;
  partitioningTimes_ = slqPtr->partitioningTimes_;

  rewindCounter_ = slqPtr->rewindCounter_;

  eventTimes_ = slqPtr->getLogicRulesPtr()->eventTimes();
  // subsystemsSequence_ = slqPtr->getLogicRulesPtr()->subsystemsSequence();

  // optimized controller
  optimizedControllersStock_ = slqPtr->nominalControllersStock_;

  // nominal trajectories (LQ approximation is around the cached trajectories)
  nominalPostEventIndicesStock_ = slqPtr->cachedPostEventIndicesStock_;
  nominalTimeTrajectoriesStock_ = slqPtr->cachedTimeTrajectoriesStock_;
  nominalStateTrajectoriesStock_ = slqPtr->cachedStateTrajectoriesStock_;
  nominalInputTrajectoriesStock_ = slqPtr->cachedInputTrajectoriesStock_;

  /*
   * Data which can be swapped. Note that these variables should have correct size.
   * Otherwise use setOptimizer() to construct them with correct size
   */
  // model data trajectory
  modelDataTrajectoriesStock_.swap(slqPtr->cachedModelDataTrajectoriesStock_);

  // event times model data
  modelDataEventTimesStock_.swap(slqPtr->cachedModelDataEventTimesStock_);

  // projected model data trajectory
  projectedModelDataTrajectoriesStock_.swap(slqPtr->cachedProjectedModelDataTrajectoriesStock_);

  // constrained projected variables
  RmInverseTrajectoriesStock_.swap(slqPtr->RmInverseTrajectoryStock_);
  RmInvConstrainedCholTrajectoryStock_.swap(slqPtr->RmInvConstrainedCholTrajectoryStock_);
  DmDagerTrajectoriesStock_.swap(slqPtr->DmDagerTrajectoryStock_);

  // terminal cost which is interpreted as the Heuristic function
  sHeuristics_ = slqPtr->sHeuristics_;
  SvHeuristics_.swap(slqPtr->SvHeuristics_);
  SmHeuristics_.swap(slqPtr->SmHeuristics_);

  // Riccati coefficients
  SsTimeTrajectoriesStock_.swap(slqPtr->SsTimeTrajectoryStock_);
  SsNormalizedTimeTrajectoriesStock_.swap(slqPtr->SsNormalizedTimeTrajectoryStock_);
  SsNormalizedEventsPastTheEndIndecesStock_.swap(slqPtr->SsNormalizedEventsPastTheEndIndecesStock_);
  SmTrajectoriesStock_.swap(slqPtr->SmTrajectoryStock_);
  SvTrajectoriesStock_.swap(slqPtr->SvTrajectoryStock_);
  SveTrajectoriesStock_.swap(slqPtr->SveTrajectoryStock_);
  sTrajectoriesStock_.swap(slqPtr->sTrajectoryStock_);

  // state-input constraints derivatives w.r.t. to the event times
  calculateStateInputConstraintsSensitivity(constSlqPtr, nominalTimeTrajectoriesStock_, nominalStateTrajectoriesStock_,
                                            nominalInputTrajectoriesStock_, EvDevEventTimesTrajectoryStockSet_,
                                            EvDevEventTimesProjectedTrajectoryStockSet_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_DataCollector<STATE_DIM, INPUT_DIM>::calculateStateInputConstraintsSensitivity(
    const slq_t* constSlqPtr, const std::vector<scalar_array_t>& timeTrajectoriesStock, const state_vector_array2_t& stateTrajectoriesStock,
    const input_vector_array2_t& inputTrajectoriesStock, dynamic_vector_array3_t& EvDevEventTimesTrajectoryStockSet,
    input_vector_array3_t& EvDevEventTimesProjectedTrajectoriesStockSet) {
  auto* slqPtr = const_cast<slq_t*>(constSlqPtr);

  const size_t numEventTimes = constSlqPtr->getLogicRulesPtr()->getNumEventTimes();

  // resizing EvDev container
  EvDevEventTimesTrajectoryStockSet.resize(numEventTimes);
  for (auto& EvDevEventTimesTrajectoryStock : EvDevEventTimesTrajectoryStockSet) {
    EvDevEventTimesTrajectoryStock.resize(constSlqPtr->numPartitions_);
    for (size_t i = 0; i < constSlqPtr->numPartitions_; i++) {
      EvDevEventTimesTrajectoryStock[i].resize(timeTrajectoriesStock[i].size());
    }  // end of i loop
  }
  // resizing EvDevProjected container
  EvDevEventTimesProjectedTrajectoriesStockSet.resize(numEventTimes);
  for (input_vector_array2_t& EvDevEventTimesProjectedTrajectoriesStock : EvDevEventTimesProjectedTrajectoriesStockSet) {
    EvDevEventTimesProjectedTrajectoriesStock.resize(constSlqPtr->numPartitions_);
    for (size_t i = 0; i < constSlqPtr->numPartitions_; i++) {
      EvDevEventTimesProjectedTrajectoriesStock[i].resize(timeTrajectoriesStock[i].size());
    }  // end of i loop
  }

  for (size_t i = 0; i < constSlqPtr->numPartitions_; i++) {
    for (size_t k = 0; k < timeTrajectoriesStock[i].size(); k++) {
      // set
      systemConstraintsPtr_->setCurrentStateAndControl(timeTrajectoriesStock[i][k], stateTrajectoriesStock[i][k],
                                                       inputTrajectoriesStock[i][k]);

      // evaluation
      constraint1_vector_array_t g1DevArray(numEventTimes);
      auto nc1 = systemConstraintsPtr_->numStateInputConstraint(timeTrajectoriesStock[i][k]);
      systemConstraintsPtr_->getConstraint1DerivativesEventTimes(g1DevArray);

      // if derivatives where available
      if (g1DevArray.size() > 0) {
        if (g1DevArray.size() != numEventTimes) {
          throw std::runtime_error("Incorrect array dimension for constraint1 derivatives w.r.t. event times.");
        }

        for (size_t j = 0; j < numEventTimes; j++) {
          EvDevEventTimesTrajectoryStockSet[j][i][k] = g1DevArray[j].head(nc1);
          EvDevEventTimesProjectedTrajectoriesStockSet[j][i][k] =
              DmDagerTrajectoriesStock_[i][k] * EvDevEventTimesTrajectoryStockSet[j][i][k];
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
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_DataCollector<STATE_DIM, INPUT_DIM>::resizeDataContainer(const size_t& numPartitions) {
  if (numPartitions == 0) {
    throw std::runtime_error("The number of Partitions cannot be zero!");
  }

  /*
   * Data which should be copied
   */
  // optimized controller
  optimizedControllersStock_.resize(numPartitions);

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

  // constrained projected variables
  RmInverseTrajectoriesStock_.resize(numPartitions);
  RmInvConstrainedCholTrajectoryStock_.resize(numPartitions);
  DmDagerTrajectoriesStock_.resize(numPartitions);

  // Riccati coefficients
  SsTimeTrajectoriesStock_.resize(numPartitions);
  SsNormalizedTimeTrajectoriesStock_.resize(numPartitions);
  SsNormalizedEventsPastTheEndIndecesStock_.resize(numPartitions);
  SmTrajectoriesStock_.resize(numPartitions);
  SvTrajectoriesStock_.resize(numPartitions);
  SveTrajectoriesStock_.resize(numPartitions);
  sTrajectoriesStock_.resize(numPartitions);
}

}  // namespace ocs2
