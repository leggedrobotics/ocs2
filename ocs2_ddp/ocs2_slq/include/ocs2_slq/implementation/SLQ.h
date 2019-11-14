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

#include "ocs2_slq/SLQ.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
SLQ<STATE_DIM, INPUT_DIM>::SLQ(const rollout_base_t* rolloutPtr, const derivatives_base_t* systemDerivativesPtr,
                               const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr,
                               const operating_trajectories_base_t* operatingTrajectoriesPtr,
                               const SLQ_Settings& settings /*= SLQ_Settings()*/,
                               std::shared_ptr<HybridLogicRules> logicRulesPtr /*= nullptr*/,
                               const cost_function_base_t* heuristicsFunctionPtr /*= nullptr*/)

    : BASE(rolloutPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr, settings,
           std::move(logicRulesPtr), heuristicsFunctionPtr) {
  Eigen::initParallel();
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::lineSearch(bool computeISEs) {
  // perform one rollout while the input correction for the type-1 constraint is considered.
  BASE::lineSearchBase(computeISEs);

  BASE::lsComputeISEs_ = computeISEs;
  baselineTotalCost_ = BASE::nominalTotalCost_;
  BASE::learningRateStar_ = 0.0;                                   // input correction learning rate is zero
  BASE::initLScontrollersStock_ = BASE::nominalControllersStock_;  // this will serve to init the workers

  // if no line search
  if (BASE::ddpSettings_.maxLearningRate_ < OCS2NumericTraits<scalar_t>::limitEpsilon()) {
    // clear the feedforward increments
    for (size_t i = 0; i < BASE::numPartitions_; i++) {
      BASE::nominalControllersStock_[i].deltaBiasArray_.clear();
    }
    // display
    if (BASE::ddpSettings_.displayInfo_) {
      std::cerr << "The chosen learningRate is: " << BASE::learningRateStar_ << std::endl;
    }

    return;
  }

  const auto maxNumOfLineSearches =
      static_cast<size_t>(std::log(BASE::ddpSettings_.minLearningRate_ / BASE::ddpSettings_.maxLearningRate_) /
                              std::log(BASE::ddpSettings_.lineSearchContractionRate_) +
                          1);

  alphaExpNext_ = 0;
  alphaProcessed_ = std::vector<bool>(maxNumOfLineSearches, false);

  nextTaskId_ = 0;
  std::function<void(void)> task = [this] { executeLineSearchWorker(); };
  BASE::runParallel(task, BASE::ddpSettings_.nThreads_);

  // revitalize all integrators
  event_handler_t::deactivateKillIntegration();

  // clear the feedforward increments
  for (size_t i = 0; i < BASE::numPartitions_; i++) {
    BASE::nominalControllersStock_[i].deltaBiasArray_.clear();
  }

  // display
  if (BASE::ddpSettings_.displayInfo_) {
    std::cerr << "The chosen learningRate is: " + std::to_string(BASE::learningRateStar_) << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::approximatePartitionLQ(size_t partitionIndex) {
  size_t N = BASE::nominalTimeTrajectoriesStock_[partitionIndex].size();

  if (N == 0) {
    return;  // nothing to do
  }

  nextTimeIndex_ = 0;
  nextTaskId_ = 0;
  std::function<void(void)> task = [this, partitionIndex] { executeApproximatePartitionLQWorker(partitionIndex); };
  BASE::runParallel(task, BASE::ddpSettings_.nThreads_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::executeApproximatePartitionLQWorker(size_t partitionIndex) {
  int N = BASE::nominalTimeTrajectoriesStock_[partitionIndex].size();
  int completedCount = 0;
  int timeIndex;
  size_t taskId = nextTaskId_++;  // assign task ID (atomic)

  // get next time index is atomic
  while ((timeIndex = nextTimeIndex_++) < N) {
    // execute approximateLQ for the given partition and time node index
    BASE::approximateLQWorker(taskId, partitionIndex, timeIndex);

    // increment the number of completed nodes
    completedCount++;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::calculatePartitionController(size_t partitionIndex) {
  size_t N = BASE::SsTimeTrajectoryStock_[partitionIndex].size();

  if (N == 0) {
    return;  // nothing to do
  }

  nextTimeIndex_ = 0;
  nextTaskId_ = 0;
  std::function<void(void)> task = [this, partitionIndex] { executeCalculatePartitionController(partitionIndex); };
  BASE::runParallel(task, BASE::ddpSettings_.nThreads_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::executeCalculatePartitionController(size_t partitionIndex) {
  int N = BASE::SsTimeTrajectoryStock_[partitionIndex].size();
  int timeIndex;
  size_t taskId = nextTaskId_++;  // assign task ID (atomic)

  // get next time index (atomic)
  while ((timeIndex = nextTimeIndex_++) < N) {
    BASE::calculateControllerWorker(taskId, partitionIndex, timeIndex);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::executeLineSearchWorker() {
  size_t taskId = nextTaskId_++;  // assign task ID (atomic)

  // local search forward simulation's variables
  scalar_t lsTotalCost;
  scalar_t lsConstraint1ISE, lsConstraint2ISE, lsInequalityConstraintPenalty, lsInequalityConstraintISE;
  scalar_t lsConstraint1MaxNorm, lsConstraint2MaxNorm;
  linear_controller_array_t lsControllersStock(BASE::numPartitions_);
  scalar_array2_t lsTimeTrajectoriesStock(BASE::numPartitions_);
  size_array2_t lsPostEventIndicesStock(BASE::numPartitions_);
  state_vector_array2_t lsStateTrajectoriesStock(BASE::numPartitions_);
  input_vector_array2_t lsInputTrajectoriesStock(BASE::numPartitions_);

  while (true) {
    size_t alphaExp = alphaExpNext_++;
    scalar_t learningRate = BASE::maxLearningRate_ * std::pow(BASE::ddpSettings_.lineSearchContractionRate_, alphaExp);

    /*
     * finish this thread's task since the learning rate is less than the minimum learning rate.
     * This means that the all the line search tasks are already processed or they are under
     * process in other threads.
     */
    if (!numerics::almost_ge(learningRate, BASE::ddpSettings_.minLearningRate_)) {
      break;
    }

    // skip if the current learning rate is less than the best candidate
    if (learningRate < BASE::learningRateStar_) {
      // display
      if (BASE::ddpSettings_.displayInfo_) {
        std::string linesearchDisplay;
        linesearchDisplay = "\t [Thread " + std::to_string(taskId) + "] rollout with learningRate " + std::to_string(learningRate) +
                            " is skipped: A larger learning rate is already found!";
        BASE::printString(linesearchDisplay);
      }
      break;
    }

    // do a line search
    lsControllersStock = BASE::initLScontrollersStock_;
    BASE::lineSearchWorker(taskId, learningRate, lsTotalCost, lsConstraint1ISE, lsConstraint1MaxNorm, lsConstraint2ISE,
                           lsConstraint2MaxNorm, lsInequalityConstraintPenalty, lsInequalityConstraintISE, lsControllersStock,
                           lsTimeTrajectoriesStock, lsPostEventIndicesStock, lsStateTrajectoriesStock, lsInputTrajectoriesStock);

    bool terminateLinesearchTasks = false;
    {
      std::lock_guard<std::mutex> lock(lineSearchResultMutex_);

      /*
       * based on the "greedy learning rate selection" policy:
       * cost should be better than the baseline cost but learning rate should
       * be as high as possible. This is equivalent to a single core line search.
       */
      if (lsTotalCost < (baselineTotalCost_ * (1 - 1e-3 * learningRate)) && learningRate > BASE::learningRateStar_) {
        BASE::nominalTotalCost_ = lsTotalCost;
        BASE::learningRateStar_ = learningRate;
        BASE::nominalConstraint1ISE_ = lsConstraint1ISE;
        BASE::nominalConstraint1MaxNorm_ = lsConstraint1MaxNorm;
        BASE::nominalConstraint2ISE_ = lsConstraint2ISE;
        BASE::nominalConstraint2MaxNorm_ = lsConstraint2MaxNorm;
        BASE::nominalInequalityConstraintPenalty_ = lsInequalityConstraintPenalty;
        BASE::nominalInequalityConstraintISE_ = lsInequalityConstraintISE;

        BASE::nominalControllersStock_.swap(lsControllersStock);
        BASE::nominalTimeTrajectoriesStock_.swap(lsTimeTrajectoriesStock);
        BASE::nominalPostEventIndicesStock_.swap(lsPostEventIndicesStock);
        BASE::nominalStateTrajectoriesStock_.swap(lsStateTrajectoriesStock);
        BASE::nominalInputTrajectoriesStock_.swap(lsInputTrajectoriesStock);

        // whether to stop all other thread.
        terminateLinesearchTasks = true;
        for (size_t i = 0; i < alphaExp; i++) {
          if (!alphaProcessed_[i]) {
            terminateLinesearchTasks = false;
            break;
          }
        }  // end of i loop

      }  // end of if

      alphaProcessed_[alphaExp] = true;

    }  // end lock

    // kill other ongoing line search tasks
    if (terminateLinesearchTasks) {
      event_handler_t::activateKillIntegration();  // kill all integrators
      if (BASE::ddpSettings_.displayInfo_) {
        BASE::printString("\t LS: interrupt other rollout's integrations.");
      }
      break;
    }

  }  // end of while loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
typename SLQ<STATE_DIM, INPUT_DIM>::scalar_t SLQ<STATE_DIM, INPUT_DIM>::solveSequentialRiccatiEquations(const state_matrix_t& SmFinal,
                                                                                                        const state_vector_t& SvFinal,
                                                                                                        const eigen_scalar_t& sFinal) {
  BASE::SmFinalStock_[BASE::finalActivePartition_] = SmFinal;
  BASE::SvFinalStock_[BASE::finalActivePartition_] = SvFinal;
  BASE::SveFinalStock_[BASE::finalActivePartition_].setZero();
  BASE::sFinalStock_[BASE::finalActivePartition_] = sFinal;

  // solve it sequentially for the first time when useParallelRiccatiSolverFromInitItr_ is false
  if (BASE::iteration_ == 0 && !BASE::useParallelRiccatiSolverFromInitItr_) {
    for (int i = BASE::numPartitions_ - 1; i >= 0; i--) {
      if (i < BASE::initActivePartition_ || i > BASE::finalActivePartition_) {
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

      // for each partition, there is one worker
      const size_t workerIndex = 0;

      // solve backward pass
      BASE::solveSlqRiccatiEquationsWorker(workerIndex, i, BASE::SmFinalStock_[i], BASE::SvFinalStock_[i], BASE::sFinalStock_[i],
                                           BASE::SveFinalStock_[i]);

      // set the final value for next Riccati equation
      if (i > BASE::initActivePartition_) {
        BASE::SmFinalStock_[i - 1] = BASE::SmTrajectoryStock_[i].front();
        BASE::SvFinalStock_[i - 1] = BASE::SvTrajectoryStock_[i].front();
        BASE::SveFinalStock_[i - 1] = BASE::SveTrajectoryStock_[i].front();
        BASE::sFinalStock_[i - 1] = BASE::sTrajectoryStock_[i].front();
        BASE::xFinalStock_[i - 1] = BASE::nominalStateTrajectoriesStock_[i].front();
      }
    }
  }
  // solve it in parallel if useParallelRiccatiSolverFromInitItr_ is true
  else {
    nextTaskId_ = 0;
    std::function<void(void)> task = [this] { executeRiccatiSolver(); };
    BASE::runParallel(task, BASE::ddpSettings_.nThreads_);
  }

  // total number of call
  size_t numSteps = 0;
  for (size_t i = BASE::initActivePartition_; i <= BASE::finalActivePartition_; i++) {
    numSteps += BASE::SsTimeTrajectoryStock_[i].size();
  }

  // average time step
  return (BASE::finalTime_ - BASE::initTime_) / numSteps;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::executeRiccatiSolver() {
  size_t taskId = nextTaskId_++;  // assign task ID (atomic)

  for (int i = BASE::endingIndicesRiccatiWorker_[taskId]; i >= BASE::startingIndicesRiccatiWorker_[taskId]; i--) {
    // for inactive subsystems
    if (i < BASE::initActivePartition_ || i > BASE::finalActivePartition_) {
      BASE::SsTimeTrajectoryStock_[i].clear();
      BASE::SmTrajectoryStock_[i].clear();
      BASE::SvTrajectoryStock_[i].clear();
      BASE::SveTrajectoryStock_[i].clear();
      BASE::sTrajectoryStock_[i].clear();

      {  // lock data
        std::lock_guard<std::mutex> lock(riccatiSolverDataMutex_);

        BASE::SmFinalStock_[i].setZero();
        BASE::SvFinalStock_[i].setZero();
        BASE::SveFinalStock_[i].setZero();
        BASE::sFinalStock_[i].setZero();
        BASE::xFinalStock_[i].setZero();
      }

    } else {
      state_matrix_t SmFinal;
      state_vector_t SvFinal;
      state_vector_t SveFinal;
      eigen_scalar_t sFinal;
      state_vector_t xFinal;

      {  // lock data
        std::lock_guard<std::mutex> lock(riccatiSolverDataMutex_);

        SmFinal = BASE::SmFinalStock_[i];
        SvFinal = BASE::SvFinalStock_[i];
        SveFinal = BASE::SveFinalStock_[i];
        sFinal = BASE::sFinalStock_[i];
        xFinal = BASE::xFinalStock_[i];
      }

      // modify the end subsystem final values based on the cached values for asynchronous run
      if (i == BASE::endingIndicesRiccatiWorker_[taskId] && i < BASE::finalActivePartition_) {
        const state_vector_t deltaState = BASE::nominalStateTrajectoriesStock_[i + 1].front() - xFinal;
        sFinal += deltaState.transpose() * (0.5 * SmFinal * deltaState + SvFinal);
        SvFinal += SmFinal * deltaState;
      }

      // solve the backward pass
      BASE::solveSlqRiccatiEquationsWorker(taskId, i, SmFinal, SvFinal, sFinal, SveFinal);

      // set the final value for next Riccati equation
      if (i > BASE::initActivePartition_) {
        // lock data
        std::lock_guard<std::mutex> lock(riccatiSolverDataMutex_);

        BASE::SmFinalStock_[i - 1] = BASE::SmTrajectoryStock_[i].front();
        BASE::SvFinalStock_[i - 1] = BASE::SvTrajectoryStock_[i].front();
        BASE::SveFinalStock_[i - 1] = BASE::SveTrajectoryStock_[i].front();
        BASE::sFinalStock_[i - 1] = BASE::sTrajectoryStock_[i].front();
        BASE::xFinalStock_[i - 1] = BASE::nominalStateTrajectoriesStock_[i].front();
      }
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::runInit() {
  // disable Eigen multi-threading
  Eigen::setNbThreads(1);

  BASE::runInit();

  // restore default Eigen thread number
  Eigen::setNbThreads(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::runIteration() {
  // disable Eigen multi-threading
  Eigen::setNbThreads(1);

  BASE::runIteration();

  // restore default Eigen thread number
  Eigen::setNbThreads(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::runExit() {
  // disable Eigen multi-threading
  Eigen::setNbThreads(1);

  BASE::runExit();

  // restore default Eigen thread number
  Eigen::setNbThreads(0);
}

}  // namespace ocs2
