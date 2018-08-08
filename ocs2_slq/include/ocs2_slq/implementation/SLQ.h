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

namespace ocs2{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::SLQ(
		const controlled_system_base_t* systemDynamicsPtr,
		const derivatives_base_t* systemDerivativesPtr,
		const constraint_base_t* systemConstraintsPtr,
		const cost_function_base_t* costFunctionPtr,
		const operating_trajectories_base_t* operatingTrajectoriesPtr,
		const SLQ_Settings& settings /*= SLQ_Settings()*/,
		const LOGIC_RULES_T* logicRulesPtr /*= nullptr*/,
		const cost_function_base_t* heuristicsFunctionPtr /*= nullptr*/)

	: BASE(systemDynamicsPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr,
			settings, logicRulesPtr, heuristicsFunctionPtr)
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::~SLQ()
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::approximatePartitionLQ(const size_t& partitionIndex)  {

	const size_t threadId = 0;
	size_t N = BASE::nominalTimeTrajectoriesStock_[partitionIndex].size();

	if (N > 0) {
		for (size_t k=0; k<N; k++) {
			// execute approximateLQWorker for the given partition and time node index (k)
			BASE::approximateLQWorker(threadId, partitionIndex, k);
		} // end of k loop
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculatePartitionController(const size_t& partitionIndex)  {

	const size_t threadId = 0;
	size_t N = BASE::SsTimeTrajectoryStock_[partitionIndex].size();

	if (N > 0) {

		for (size_t k=0; k<N; k++) {
			// execute calculateControllerWorker for the given partition and time node index (k)
			BASE::calculateControllerWorker(threadId, partitionIndex, k);
		} // end of k loop

	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::lineSearch(bool computeISEs)  {

	// perform one rollout while the input correction for the type-1 constraint is considered.
	BASE::lineSearchBase(computeISEs);

	const size_t workerIndex = 0;
	BASE::lsComputeISEs_ = computeISEs;
	scalar_t learningRate = BASE::maxLearningRate_;
	BASE::initLScontrollersStock_ = BASE::nominalControllersStock_;

	// local search forward simulation's variables
	scalar_t lsTotalCost;
	scalar_t lsConstraint1ISE, lsConstraint2ISE;
	scalar_t lsConstraint1MaxNorm, lsConstraint2MaxNorm;
	controller_array_t			lsControllersStock(BASE::numPartitions_);
	std::vector<scalar_array_t>	lsTimeTrajectoriesStock(BASE::numPartitions_);
	std::vector<size_array_t>	lsEventsPastTheEndIndecesStock(BASE::numPartitions_);
	state_vector_array2_t   	lsStateTrajectoriesStock(BASE::numPartitions_);
	input_vector_array2_t 		lsInputTrajectoriesStock(BASE::numPartitions_);

	while (learningRate >= BASE::settings_.minLearningRateGSLQP_)  {

		// do a line search
		lsControllersStock = BASE::initLScontrollersStock_;
		BASE::lineSearchWorker(workerIndex, learningRate,
				lsTotalCost,
				lsConstraint1ISE, lsConstraint1MaxNorm,
				lsConstraint2ISE, lsConstraint2MaxNorm,
				lsControllersStock,
				lsTimeTrajectoriesStock, lsEventsPastTheEndIndecesStock,
				lsStateTrajectoriesStock, lsInputTrajectoriesStock);

		// break condition 1: it exits with largest learningRate that its cost is smaller than nominal cost.
		if (lsTotalCost < BASE::nominalTotalCost_*(1-1e-3*learningRate))
			break;  // exit while loop
		else
			learningRate = BASE::settings_.lineSearchContractionRate_*learningRate;

	}  // end of while


	if (learningRate >= BASE::settings_.minLearningRateGSLQP_)  {
		BASE::learningRateStar_ = learningRate;
		BASE::nominalTotalCost_ = lsTotalCost;
		BASE::nominalConstraint1ISE_ 	 = lsConstraint1ISE;
		BASE::nominalConstraint1MaxNorm_ = lsConstraint1MaxNorm;
		BASE::nominalConstraint2ISE_ 	 = lsConstraint2ISE;
		BASE::nominalConstraint2MaxNorm_ = lsConstraint2MaxNorm;

		BASE::nominalControllersStock_.swap(lsControllersStock);
		BASE::nominalTimeTrajectoriesStock_.swap(lsTimeTrajectoriesStock);
		BASE::nominalEventsPastTheEndIndecesStock_.swap(lsEventsPastTheEndIndecesStock);
		BASE::nominalStateTrajectoriesStock_.swap(lsStateTrajectoriesStock);
		BASE::nominalInputTrajectoriesStock_.swap(lsInputTrajectoriesStock);

	} else // since the open loop input is not change, the nominal trajectories will be unchanged
		BASE::learningRateStar_ = 0.0;

	// clear the feedforward increments
	for (size_t i=0; i<BASE::numPartitions_; i++)
		BASE::nominalControllersStock_[i].deltaUff_.clear();

	// display
	if (BASE::settings_.displayInfo_)
		std::cerr << "The chosen learningRate is: " << BASE::learningRateStar_ << std::endl;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
typename SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_t
	SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::solveSequentialRiccatiEquations(
		const state_matrix_t& SmFinal,
		const state_vector_t& SvFinal,
		const eigen_scalar_t& sFinal)  {

	// for all partition, there is only one worker
	const size_t workerIndex = 0;

	BASE::SmFinalStock_[BASE::finalActivePartition_]  = SmFinal;
	BASE::SvFinalStock_[BASE::finalActivePartition_]  = SvFinal;
	BASE::SveFinalStock_[BASE::finalActivePartition_] = state_vector_t::Zero();
	BASE::sFinalStock_[BASE::finalActivePartition_]   = sFinal;

	for (int i=BASE::numPartitions_-1; i>=0; i--) {

		if (i<(signed)BASE::initActivePartition_ || i>(signed)BASE::finalActivePartition_) {

			BASE::SsTimeTrajectoryStock_[i].clear();
			BASE::SsNormalizedTimeTrajectoryStock_[i].clear();
			BASE::SsNormalizedEventsPastTheEndIndecesStock_[i].clear();
			BASE::SmTrajectoryStock_[i].clear();
			BASE::SvTrajectoryStock_[i].clear();
			BASE::SveTrajectoryStock_[i].clear();
			BASE::sTrajectoryStock_[i].clear();

			BASE::SmFinalStock_[i].setZero();
			BASE::SvFinalStock_[i].setZero();
			BASE::SveFinalStock_[i].setZero();
			BASE::sFinalStock_[i].setZero();
			BASE::xFinalStock_[i].setZero();

			continue;
		}

		if (BASE::settings_.useRiccatiSolver_==true) {
			BASE::solveSlqRiccatiEquationsWorker(workerIndex, i,
					BASE::SmFinalStock_[i], BASE::SvFinalStock_[i], BASE::sFinalStock_[i], BASE::SveFinalStock_[i]);
		} else {
			scalar_t constraintStepSize = BASE::initialControllerDesignStock_[i] ? 0.0 : BASE::settings_.constraintStepSize_;
			BASE::fullRiccatiBackwardSweepWorker(workerIndex, i,
					BASE::SmFinalStock_[i], BASE::SvFinalStock_[i], BASE::SveFinalStock_[i], BASE::sFinalStock_[i],
					constraintStepSize);
		}

		// set the final value for next Riccati equation
		if (i>BASE::initActivePartition_) {
			BASE::SmFinalStock_[i-1]  = BASE::SmTrajectoryStock_[i].front();
			BASE::SvFinalStock_[i-1]  = BASE::SvTrajectoryStock_[i].front();
			BASE::SveFinalStock_[i-1] = BASE::SveTrajectoryStock_[i].front();
			BASE::sFinalStock_[i-1]   = BASE::sTrajectoryStock_[i].front();
			BASE::xFinalStock_[i-1]	  = BASE::nominalStateTrajectoriesStock_[i].front();
		}
	}  // end of i loop

	// total number of call
	size_t numSteps = 0;
	for (size_t i=BASE::initActivePartition_; i<=BASE::finalActivePartition_; i++)
		numSteps += BASE::SsTimeTrajectoryStock_[i].size();

	// average time step
	return (BASE::finalTime_-BASE::initTime_)/(scalar_t)numSteps;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::runInit() {

	// run BASE routine
	BASE::runInit();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::runIteration()  {

	// run BASE routine
	BASE::runIteration();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::runExit()  {

	// run BASE routine
	BASE::runExit();
}


} // namespace ocs2
