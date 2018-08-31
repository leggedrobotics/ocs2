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
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_DataCollector<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::collect(
		const slq_t* constSlqPtr) {

	slq_t* slqPtr = const_cast<slq_t*>(constSlqPtr);

	/*
	 * Data which should be copied
	 */
	// initial time and state plus final time
	initTime_  = slqPtr->initTime_;
	finalTime_ = slqPtr->finalTime_;
	initState_ = slqPtr->initState_;

	// active partitions range: [initActivePartition_, finalActivePartition_]
	initActivePartition_  = slqPtr->initActivePartition_;
	finalActivePartition_ = slqPtr->finalActivePartition_;

	// data resizing
	bool numPartitionsChanged = numPartitions_ != slqPtr->numPartitions_;
	if (numPartitionsChanged==true)
		resizeDataContainer(slqPtr->numPartitions_);

	numPartitions_     = slqPtr->numPartitions_;
	partitioningTimes_ = slqPtr->partitioningTimes_;

	rewindCounter_ = slqPtr->rewindCounter_;

	// optimized controller
	optimizedControllersStock_ = slqPtr->nominalControllersStock_;

	// optimized trajectories
//	optimizedTimeTrajectoriesStock_        = slqPtr->nominalTimeTrajectoriesStock_;
//	optimizedEventsPastTheEndIndecesStock_ = slqPtr->nominalEventsPastTheEndIndecesStock_;
//	optimizedStateTrajectoriesStock_       = slqPtr->nominalStateTrajectoriesStock_;
//	optimizedInputTrajectoriesStock_       = slqPtr->nominalInputTrajectoriesStock_;

	/*
	 * Data which can be swapped. Note that these variables should have correct size.
	 * Otherwise use setOptimizer() to construct them with correct size
	 */
	// nominal trajectories
	nominalEventsPastTheEndIndecesStock_.swap(slqPtr->nominalPrevEventsPastTheEndIndecesStock_);
	nominalStateTrajectoriesStock_.swap(slqPtr->nominalPrevStateTrajectoriesStock_);
	nominalTimeTrajectoriesStock_.swap(slqPtr->nominalPrevTimeTrajectoriesStock_);
	nominalInputTrajectoriesStock_.swap(slqPtr->nominalPrevInputTrajectoriesStock_);

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
	RmConstrainedTrajectoriesStock_.swap(slqPtr->RmConstrainedTrajectoryStock_);
	DmDagerTrajectoriesStock_.swap(slqPtr->DmDagerTrajectoryStock_);
	EvProjectedTrajectoriesStock_.swap(slqPtr->EvProjectedTrajectoryStock_);
	CmProjectedTrajectoriesStock_.swap(slqPtr->CmProjectedTrajectoryStock_);
	DmProjectedTrajectoriesStock_.swap(slqPtr->DmProjectedTrajectoryStock_);
	BmConstrainedTrajectoriesStock_.swap(slqPtr->BmConstrainedTrajectoryStock_);
	PmConstrainedTrajectoriesStock_.swap(slqPtr->PmConstrainedTrajectoryStock_);
	RvConstrainedTrajectoriesStock_.swap(slqPtr->RvConstrainedTrajectoryStock_);

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
	calculateFlowMap(constSlqPtr,
			nominalTimeTrajectoriesStock_,
			nominalStateTrajectoriesStock_,
			nominalInputTrajectoriesStock_,
			nominalFlowMapTrajectoriesStock_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_DataCollector<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateFlowMap(
		const slq_t* constSlqPtr,
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const state_vector_array2_t& stateTrajectoriesStock,
		const input_vector_array2_t& inputTrajectoriesStock,
		state_vector_array2_t& flowMapTrajectoriesStock)  {

	const size_t workerIndex = 0;
	slq_t* slqPtr = const_cast<slq_t*>(constSlqPtr);

	flowMapTrajectoriesStock.resize(slqPtr->numPartitions_);

	for (size_t i=0; i<slqPtr->numPartitions_; i++) {

		// skip the inactive subsystems
		if (i<slqPtr->initActivePartition_ || i>slqPtr->finalActivePartition_) {
			flowMapTrajectoriesStock[i].clear();
			continue;
		}

		const size_t N = timeTrajectoriesStock[i].size();
		flowMapTrajectoriesStock[i].resize(N);

		// set controller
//		slqPtr->systemDynamicsPtrStock_[workerIndex]->setController(nominalControllersStock_[i]);
		// initialize subsystem
		slqPtr->systemDynamicsPtrStock_[workerIndex]->initializeModel(*(slqPtr->logicRulesMachinePtr_), i, "SLQ");

		for (size_t k=0; k<N; k++) {
			slqPtr->systemDynamicsPtrStock_[workerIndex]->computeFlowMap(
					timeTrajectoriesStock[i][k],
					stateTrajectoriesStock[i][k],
					inputTrajectoriesStock[i][k],
					flowMapTrajectoriesStock[i][k]);
		}  // end of k loop
	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_DataCollector<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::resizeDataContainer(
		const size_t& numPartitions) {

	if (numPartitions==0)
		throw std::runtime_error("The number of Partitions cannot be zero!");

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
	nominalEventsPastTheEndIndecesStock_.resize(numPartitions);
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
	RmConstrainedTrajectoriesStock_.resize(numPartitions);
	DmDagerTrajectoriesStock_.resize(numPartitions);
	EvProjectedTrajectoriesStock_.resize(numPartitions);
	CmProjectedTrajectoriesStock_.resize(numPartitions);
	DmProjectedTrajectoriesStock_.resize(numPartitions);
	BmConstrainedTrajectoriesStock_.resize(numPartitions);
	PmConstrainedTrajectoriesStock_.resize(numPartitions);
	RvConstrainedTrajectoriesStock_.resize(numPartitions);

	// Riccati coefficients
	SsTimeTrajectoriesStock_.resize(numPartitions);
	SsNormalizedTimeTrajectoriesStock_.resize(numPartitions);
	SsNormalizedEventsPastTheEndIndecesStock_.resize(numPartitions);
	SmTrajectoriesStock_.resize(numPartitions);
	SvTrajectoriesStock_.resize(numPartitions);
	SveTrajectoriesStock_.resize(numPartitions);
	sTrajectoriesStock_.resize(numPartitions);
}

} // namespace ocs2
