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

  eventTimes_ = slqPtr->getModeSchedule().eventTimes;
  subsystemsSequence_ = slqPtr->getModeSchedule().modeSequence;

  // optimized controller
  optimizedControllersStock_ = slqPtr->nominalControllersStock_;

  /*
   * Data which can be swapped. Note that these variables should have correct size.
   * Otherwise use setOptimizer() to construct them with correct size
   */
  // nominal trajectories
  nominalPostEventIndicesStock_.swap(slqPtr->cachedPostEventIndicesStock_);
  nominalStateTrajectoriesStock_.swap(slqPtr->cachedStateTrajectoriesStock_);
  nominalTimeTrajectoriesStock_.swap(slqPtr->cachedTimeTrajectoriesStock_);
  nominalInputTrajectoriesStock_.swap(slqPtr->cachedInputTrajectoriesStock_);

  // linearized system coefficients
  AmTrajectoriesStock_.swap(slqPtr->AmTrajectoryStock_);
  BmTrajectoriesStock_.swap(slqPtr->BmTrajectoryStock_);

  nc1TrajectoriesStock_.swap(slqPtr->nc1TrajectoriesStock_);
  EvTrajectoriesStock_.swap(slqPtr->EvTrajectoryStock_);
  CmTrajectoriesStock_.swap(slqPtr->CmTrajectoryStock_);
  DmTrajectoriesStock_.swap(slqPtr->DmTrajectoryStock_);

  nc2TrajectoriesStock_.swap(slqPtr->nc2TrajectoriesStock_);
  HvTrajectoriesStock_.swap(slqPtr->HvTrajectoryStock_);
  FmTrajectoriesStock_.swap(slqPtr->FmTrajectoryStock_);
  nc2FinalStock_.swap(slqPtr->nc2FinalStock_);
  HvFinalStock_.swap(slqPtr->HvFinalStock_);
  FmFinalStock_.swap(slqPtr->FmFinalStock_);

  ncIneqTrajectoriesStock_.swap(slqPtr->ncIneqTrajectoriesStock_);
  hTrajectoryStock_.swap(slqPtr->hTrajectoryStock_);
  dhdxTrajectoryStock_.swap(slqPtr->dhdxTrajectoryStock_);
  ddhdxdxTrajectoryStock_.swap(slqPtr->ddhdxdxTrajectoryStock_);
  dhduTrajectoryStock_.swap(slqPtr->dhduTrajectoryStock_);
  ddhduduTrajectoryStock_.swap(slqPtr->ddhduduTrajectoryStock_);
  ddhdudxTrajectoryStock_.swap(slqPtr->ddhdudxTrajectoryStock_);

  // cost quadratic approximation coefficients
  qFinalStock_.swap(slqPtr->qFinalStock_);
  QvFinalStock_.swap(slqPtr->QvFinalStock_);
  QmFinalStock_.swap(slqPtr->QmFinalStock_);

  qTrajectoriesStock_.swap(slqPtr->qTrajectoryStock_);
  QvTrajectoriesStock_.swap(slqPtr->QvTrajectoryStock_);
  QmTrajectoriesStock_.swap(slqPtr->QmTrajectoryStock_);
  RvTrajectoriesStock_.swap(slqPtr->RvTrajectoryStock_);
  RmTrajectoriesStock_.swap(slqPtr->RmTrajectoryStock_);
  PmTrajectoriesStock_.swap(slqPtr->PmTrajectoryStock_);

  // constrained projected variables
  RmInverseTrajectoriesStock_.swap(slqPtr->RmInverseTrajectoryStock_);
  AmConstrainedTrajectoriesStock_.swap(slqPtr->AmConstrainedTrajectoryStock_);
  QmConstrainedTrajectoriesStock_.swap(slqPtr->QmConstrainedTrajectoryStock_);
  QvConstrainedTrajectoriesStock_.swap(slqPtr->QvConstrainedTrajectoryStock_);
  RmInvConstrainedCholTrajectoryStock_.swap(slqPtr->RmInvConstrainedCholTrajectoryStock_);
  DmDagerTrajectoriesStock_.swap(slqPtr->DmDagerTrajectoryStock_);
  EvProjectedTrajectoriesStock_.swap(slqPtr->EvProjectedTrajectoryStock_);
  CmProjectedTrajectoriesStock_.swap(slqPtr->CmProjectedTrajectoryStock_);
  DmProjectedTrajectoriesStock_.swap(slqPtr->DmProjectedTrajectoryStock_);

  // terminal cost which is interpreted as the Heuristic function
  sHeuristics_.swap(slqPtr->sHeuristics_);
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

  // SLQ missing variables flow-map value
  calculateFlowMap(constSlqPtr, nominalTimeTrajectoriesStock_, nominalStateTrajectoriesStock_, nominalInputTrajectoriesStock_,
                   nominalFlowMapTrajectoriesStock_);

  // state-input constraints derivatives w.r.t. to the event times
  calculateStateInputConstraintsSensitivity(constSlqPtr, nominalTimeTrajectoriesStock_, nominalStateTrajectoriesStock_,
                                            nominalInputTrajectoriesStock_, EvDevEventTimesTrajectoriesStockSet_,
                                            EvDevEventTimesProjectedTrajectoriesStockSet_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_DataCollector<STATE_DIM, INPUT_DIM>::calculateFlowMap(const slq_t* constSlqPtr,
                                                               const std::vector<scalar_array_t>& timeTrajectoriesStock,
                                                               const state_vector_array2_t& stateTrajectoriesStock,
                                                               const input_vector_array2_t& inputTrajectoriesStock,
                                                               state_vector_array2_t& flowMapTrajectoriesStock) {
  auto* slqPtr = const_cast<slq_t*>(constSlqPtr);

  flowMapTrajectoriesStock.resize(constSlqPtr->numPartitions_);

  for (size_t i = 0; i < constSlqPtr->numPartitions_; i++) {
    // skip the inactive subsystems
    if (i < constSlqPtr->initActivePartition_ || i > constSlqPtr->finalActivePartition_) {
      flowMapTrajectoriesStock[i].clear();
      continue;
    }

    const size_t N = timeTrajectoriesStock[i].size();
    flowMapTrajectoriesStock[i].resize(N);

    auto timeTriggeredRolloutPtr = dynamic_cast<TimeTriggeredRollout<STATE_DIM, INPUT_DIM>*>(rolloutPtr_.get());
    if (!timeTriggeredRolloutPtr) {
      throw std::runtime_error("The Rollout pointer provided to SLQ_DataCollector is not of type TimeTriggeredRollout.");
    }

    // set controller
    timeTriggeredRolloutPtr->systemDynamicsPtr()->setController(&(slqPtr->nominalControllersStock_[i]));

    for (size_t k = 0; k < N; k++) {
      timeTriggeredRolloutPtr->systemDynamicsPtr()->computeFlowMap(timeTrajectoriesStock[i][k], stateTrajectoriesStock[i][k],
                                                                   inputTrajectoriesStock[i][k], flowMapTrajectoriesStock[i][k]);
    }  // end of k loop
  }    // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_DataCollector<STATE_DIM, INPUT_DIM>::calculateStateInputConstraintsSensitivity(
    const slq_t* constSlqPtr, const std::vector<scalar_array_t>& timeTrajectoriesStock, const state_vector_array2_t& stateTrajectoriesStock,
    const input_vector_array2_t& inputTrajectoriesStock, constraint1_vector_array3_t& EvDevEventTimesTrajectoriesStockSet,
    input_vector_array3_t& EvDevEventTimesProjectedTrajectoriesStockSet) {
  auto* slqPtr = const_cast<slq_t*>(constSlqPtr);

  const size_t numEventTimes = constSlqPtr->getModeSchedule().eventTimes.size();

  // resizing EvDev container
  EvDevEventTimesTrajectoriesStockSet.resize(numEventTimes);
  for (constraint1_vector_array2_t& EvDevEventTimesTrajectoriesStock : EvDevEventTimesTrajectoriesStockSet) {
    EvDevEventTimesTrajectoriesStock.resize(constSlqPtr->numPartitions_);
    for (size_t i = 0; i < constSlqPtr->numPartitions_; i++) {
      EvDevEventTimesTrajectoriesStock[i].resize(timeTrajectoriesStock[i].size());
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
      systemConstraintsPtr_->getConstraint1DerivativesEventTimes(g1DevArray);

      // if derivatives where available
      if (g1DevArray.size() > 0) {
        if (g1DevArray.size() != numEventTimes) {
          throw std::runtime_error("Incorrect array dimension for constraint1 derivatives w.r.t. event times.");
        }

        for (size_t j = 0; j < numEventTimes; j++) {
          EvDevEventTimesTrajectoriesStockSet[j][i][k].swap(g1DevArray[j]);
          EvDevEventTimesProjectedTrajectoriesStockSet[j][i][k] =
              DmDagerTrajectoriesStock_[i][k] * EvDevEventTimesTrajectoriesStockSet[j][i][k];
        }  // end of j loop

      } else {
        for (size_t j = 0; j < numEventTimes; j++) {
          EvDevEventTimesTrajectoriesStockSet[j][i][k].setZero();
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

  // optimized trajectories
  //	optimizedTimeTrajectoriesStock_.resize(numPartitions);
  //	optimizedEventsPastTheEndIndecesStock_.resize(numPartitions);
  //	optimizedStateTrajectoriesStock_.resize(numPartitions);
  //	optimizedInputTrajectoriesStock_.resize(numPartitions);

  // nominal trajectories
  nominalTimeTrajectoriesStock_.resize(numPartitions);
  nominalPostEventIndicesStock_.resize(numPartitions);
  nominalStateTrajectoriesStock_.resize(numPartitions);
  nominalInputTrajectoriesStock_.resize(numPartitions);

  /*
   * Data which can be swapped. Note that these variables should have correct size.
   * Otherwise use setOptimizer() to construct them with correct size
   */
  // linearized system coefficients
  AmTrajectoriesStock_.resize(numPartitions);
  BmTrajectoriesStock_.resize(numPartitions);

  nc1TrajectoriesStock_.resize(numPartitions);
  EvTrajectoriesStock_.resize(numPartitions);
  CmTrajectoriesStock_.resize(numPartitions);
  DmTrajectoriesStock_.resize(numPartitions);

  nc2TrajectoriesStock_.resize(numPartitions);
  HvTrajectoriesStock_.resize(numPartitions);
  FmTrajectoriesStock_.resize(numPartitions);
  nc2FinalStock_.resize(numPartitions);
  HvFinalStock_.resize(numPartitions);
  FmFinalStock_.resize(numPartitions);

  ncIneqTrajectoriesStock_.resize(numPartitions);
  hTrajectoryStock_.resize(numPartitions);
  dhdxTrajectoryStock_.resize(numPartitions);
  ddhdxdxTrajectoryStock_.resize(numPartitions);
  dhduTrajectoryStock_.resize(numPartitions);
  ddhduduTrajectoryStock_.resize(numPartitions);
  ddhdudxTrajectoryStock_.resize(numPartitions);

  // cost quadratic approximation coefficients
  qFinalStock_.resize(numPartitions);
  QvFinalStock_.resize(numPartitions);
  QmFinalStock_.resize(numPartitions);

  qTrajectoriesStock_.resize(numPartitions);
  QvTrajectoriesStock_.resize(numPartitions);
  QmTrajectoriesStock_.resize(numPartitions);
  RvTrajectoriesStock_.resize(numPartitions);
  RmTrajectoriesStock_.resize(numPartitions);
  PmTrajectoriesStock_.resize(numPartitions);

  // constrained projected variables
  RmInverseTrajectoriesStock_.resize(numPartitions);
  AmConstrainedTrajectoriesStock_.resize(numPartitions);
  QmConstrainedTrajectoriesStock_.resize(numPartitions);
  QvConstrainedTrajectoriesStock_.resize(numPartitions);
  RmInvConstrainedCholTrajectoryStock_.resize(numPartitions);
  DmDagerTrajectoriesStock_.resize(numPartitions);
  EvProjectedTrajectoriesStock_.resize(numPartitions);
  CmProjectedTrajectoriesStock_.resize(numPartitions);
  DmProjectedTrajectoriesStock_.resize(numPartitions);

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
