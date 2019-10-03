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

#include "ocs2_slq/SLQ_MP.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
SLQ_MP<STATE_DIM, INPUT_DIM>::SLQ_MP(const controlled_system_base_t* systemDynamicsPtr, const derivatives_base_t* systemDerivativesPtr,
                                     const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr,
                                     const operating_trajectories_base_t* operatingTrajectoriesPtr,
                                     const SLQ_Settings& settings /*= SLQ_Settings()*/,
                                     std::shared_ptr<HybridLogicRules> logicRulesPtr /*= nullptr*/,
                                     const cost_function_base_t* heuristicsFunctionPtr /*= nullptr*/)

    : BASE(systemDynamicsPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr, settings,
           std::move(logicRulesPtr), heuristicsFunctionPtr),
      workerTask_(IDLE),
      subsystemProcessed_(0),
      workerException_(nullptr) {
  Eigen::initParallel();

  // initialize threads
  launchWorkerThreads();
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
SLQ_MP<STATE_DIM, INPUT_DIM>::~SLQ_MP() {
  workersActive_ = false;
  {
    std::lock_guard<std::mutex> waitLock(workerWakeUpMutex_);
    workerTask_ = SHUTDOWN;
  }
  workerWakeUpCondition_.notify_all();

  if (BASE::ddpSettings_.debugPrintMT_) {
    std::cerr << "Shutting down workers." << std::endl;
  }

  for (size_t i = 0; i < workerThreads_.size(); i++) {
    workerThreads_[i].join();
  }

  if (BASE::ddpSettings_.debugPrintMT_) {
    std::cerr << "All workers shut down" << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_MP<STATE_DIM, INPUT_DIM>::lineSearch(bool computeISEs) {
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

  {
    std::lock_guard<std::mutex> waitLock(workerWakeUpMutex_);
    subsystemProcessed_ = 0;  // not required for linesearch, but assign to not let it dangle around
  }

  alphaProcessed_.clear();
  alphaTaken_ = 0;
  alphaBestFound_ = false;

  {
    std::lock_guard<std::mutex> waitLock(alphaBestFoundMutex_);
    lsWorkerCompleted_ = 0;
  }

  auto maxNumOfLineSearches = static_cast<size_t>(std::log(BASE::ddpSettings_.minLearningRate_ / BASE::ddpSettings_.maxLearningRate_) /
                                                      std::log(BASE::ddpSettings_.lineSearchContractionRate_) +
                                                  1);

  alphaExpMax_ = maxNumOfLineSearches;
  alphaExpBest_ = maxNumOfLineSearches;
  alphaProcessed_ = std::vector<bool>(maxNumOfLineSearches, false);

  if (BASE::ddpSettings_.debugPrintMT_) {
    std::cerr << "[MT]: calculated maximum number of line searches " + std::to_string(alphaExpMax_) << std::endl;
    std::cerr << "[MT]: Waking up workers for line search " << std::endl;
  }

  {
    std::lock_guard<std::mutex> waitLock(workerWakeUpMutex_);
    workerTask_ = LINE_SEARCH;
  }
  workerWakeUpCondition_.notify_all();

  if (BASE::ddpSettings_.debugPrintMT_) {
    BASE::printString("[MT]: Will sleep now until we have results ");
  }

  {
    std::unique_lock<std::mutex> waitLock(alphaBestFoundMutex_);
    alphaBestFoundCondition_.wait(waitLock, [&] { return lsWorkerCompleted_ >= BASE::ddpSettings_.nThreads_; });
  }

  {
    std::lock_guard<std::mutex> waitLock(workerWakeUpMutex_);
    workerTask_ = IDLE;
  }

  // revitalize all integrator
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
void SLQ_MP<STATE_DIM, INPUT_DIM>::launchWorkerThreads() {
  workersActive_ = true;
  {
    std::lock_guard<std::mutex> waitLock(workerWakeUpMutex_);
    workerTask_ = IDLE;
  }

  workerThreads_.clear();
  for (size_t i = 0; i < BASE::ddpSettings_.nThreads_; i++) {
    workerThreads_.push_back(std::thread(&SLQ_MP::threadWork, this, i));
    setThreadPriority(BASE::ddpSettings_.threadPriority_, workerThreads_[i]);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_MP<STATE_DIM, INPUT_DIM>::threadWork(size_t threadId) {
  if (BASE::ddpSettings_.debugPrintMT_) {
    BASE::printString("[MT]: [Thread " + std::to_string(threadId) + "]: launched");
  }

  // local variables
  size_t uniqueProcessID = 0;
  size_t subsystemProcessed_local = 0;
  size_t iteration_local = BASE::iteration_;
  int workerTask_local = IDLE;

  while (workersActive_) {
    subsystemProcessed_local = subsystemProcessed_.load();
    workerTask_local = workerTask_.load();
    iteration_local = BASE::iteration_;

    // display
    if (BASE::ddpSettings_.debugPrintMT_) {
      BASE::printString(
          "[MT]: [Thread " + std::to_string(threadId) + "]: previous procId: " + std::to_string(uniqueProcessID) +
          ", current procId: " + std::to_string(generateUniqueProcessID(iteration_local, workerTask_local, subsystemProcessed_local)));
    }

    /* We want to put the worker to sleep if
     * - the workerTask_ is IDLE
     * - or we are finished both workerTask_ is not yet reset, thus the process ID is still the same
     * */
    if (workerTask_local == IDLE ||
        uniqueProcessID == generateUniqueProcessID(iteration_local, workerTask_local, subsystemProcessed_local)) {
      if (BASE::ddpSettings_.debugPrintMT_) {
        BASE::printString("[MT]: [Thread " + std::to_string(threadId) + "]: going to sleep !");
      }

      // sleep until the state is not IDLE any more and we have a different process ID than before
      {
        std::unique_lock<std::mutex> waitLock(workerWakeUpMutex_);
        workerWakeUpCondition_.wait(waitLock, [&] {
          return (workerTask_ != IDLE) && (uniqueProcessID != generateUniqueProcessID(BASE::iteration_, workerTask_, subsystemProcessed_));
        });
      }

      subsystemProcessed_local = subsystemProcessed_.load();
      workerTask_local = workerTask_.load();
      iteration_local = BASE::iteration_;

      if (BASE::ddpSettings_.debugPrintMT_) {
        BASE::printString("[MT]: [Thread " + std::to_string(threadId) + "]: woke up !");
      }
    }

    if (!workersActive_) {
      break;
    }

    try {
      switch (workerTask_local) {
        case APPROXIMATE_LQ: {
          if (BASE::ddpSettings_.debugPrintMT_) {
            BASE::printString("[MT]: [Thread " + std::to_string(threadId) + "]: is busy with APPROXIMATE_LQ on partition " +
                              std::to_string(subsystemProcessed_local));
          }

          executeApproximatePartitionLQWorker(threadId, subsystemProcessed_local);
          uniqueProcessID = generateUniqueProcessID(iteration_local, APPROXIMATE_LQ, subsystemProcessed_local);

          break;
        }
        case CALCULATE_CONTROLLER: {
          if (BASE::ddpSettings_.debugPrintMT_) {
            BASE::printString("[MT]: [Thread " + std::to_string(threadId) + "]: now busy with CALCULATE_CONTROLLER !");
          }

          executeCalculatePartitionController(threadId, subsystemProcessed_local);
          uniqueProcessID = generateUniqueProcessID(iteration_local, CALCULATE_CONTROLLER, subsystemProcessed_local);

          break;
        }
        case LINE_SEARCH: {
          if (BASE::ddpSettings_.debugPrintMT_) {
            BASE::printString("[MT]: [Thread " + std::to_string(threadId) + "]: now busy with LINE_SEARCH !");
          }

          executeLineSearchWorker(threadId);
          uniqueProcessID = generateUniqueProcessID(iteration_local, LINE_SEARCH, subsystemProcessed_local);
          break;
        }
        case SOLVE_RICCATI: {
          if (BASE::ddpSettings_.debugPrintMT_) {
            BASE::printString("[MT]: [Thread " + std::to_string(threadId) + "]: now busy with RiccatiSolver!");
          }
          uniqueProcessID = generateUniqueProcessID(iteration_local, SOLVE_RICCATI, subsystemProcessed_local);
          executeRiccatiSolver(threadId);
          break;
        }
        case SHUTDOWN: {
          if (BASE::ddpSettings_.debugPrintMT_) {
            BASE::printString("[MT]: [Thread " + std::to_string(threadId) + "]: now shutting down!");
          }
          return;
        }
      }
    } catch (...) {
      std::cerr << "Caught runtime error while doing thread work (workerTask_local " << workerTask_local << ")" << std::endl;
      {
        std::lock_guard<std::mutex> lock(workerExceptionMutex_);
        workerException_ = std::current_exception();
      }

      std::lock_guard<std::mutex> lock(riccatiSolverBarrierMutex_);
      riccatiSolverCompletedCondition_.notify_one();
    }

    if (BASE::ddpSettings_.debugPrintMT_) {
      BASE::printString("[MT]: [Thread " + std::to_string(threadId) + "]: done with job. Will wait for next now!");
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_MP<STATE_DIM, INPUT_DIM>::approximatePartitionLQ(size_t partitionIndex) {
  {
    std::lock_guard<std::mutex> waitLock(workerWakeUpMutex_);
    subsystemProcessed_ = partitionIndex;
  }

  size_t N = BASE::nominalTimeTrajectoriesStock_[partitionIndex].size();

  if (N > 0) {
    // display
    if (BASE::ddpSettings_.debugPrintMT_) {
      std::cerr << "[MT]: Activating threads to perform LQ approximation for partition " + std::to_string(partitionIndex) << std::endl;
    }

    kTaken_approx_[partitionIndex] = 0;
    kCompleted_approx_[partitionIndex] = 0;

    // activates all threads' APPROXIMATE_LQ task which in turn runs executeApproximatePartitionLQWorker routine
    {
      std::lock_guard<std::mutex> waitLock(workerWakeUpMutex_);
      workerTask_ = APPROXIMATE_LQ;
    }
    workerWakeUpCondition_.notify_all();

    // display
    if (BASE::ddpSettings_.debugPrintMT_) {
      BASE::printString("[MT]: Waiting until threads finish LQ approximation for partition " + std::to_string(partitionIndex));
    }

    // wait until all threads finish their task
    std::unique_lock<std::mutex> waitLock(kCompletedMutex_);
    while (kCompleted_approx_[partitionIndex].load() < N) {
      kCompletedCondition_.wait(waitLock);
    }
    waitLock.unlock();

    // reset threads to no task mode
    {
      std::lock_guard<std::mutex> waitLock(workerWakeUpMutex_);
      workerTask_ = IDLE;
    }

    // display
    if (BASE::ddpSettings_.debugPrintMT_) {
      std::cerr << "[MT]: Back to main thread, workers should now have finished LQ approximation for partition " +
                       std::to_string(partitionIndex)
                << std::endl;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_MP<STATE_DIM, INPUT_DIM>::executeApproximatePartitionLQWorker(size_t threadId, size_t partitionIndex) {
  size_t kCompleted_local = 0;
  int N = BASE::nominalTimeTrajectoriesStock_[partitionIndex].size();
  int k = -1;  // to make use that the while loop runs at least once

  while (k + 1 < N) {
    // update k
    k = kTaken_approx_[partitionIndex]++;

    if (k < N) {
      // display
      if (BASE::ddpSettings_.debugPrintMT_ && k % 10 == 0) {
        BASE::printString("[MT]: [Thread " + std::to_string(threadId) + "], partition " + std::to_string(partitionIndex) +
                          ":Start approximating system LQ on index k = " + std::to_string(k) + " out of " + std::to_string(N - 1));
      }

      // execute approximateLQ for the given partition and time node index (k)
      BASE::approximateLQWorker(threadId, partitionIndex, k);

      // increment the number of completed nodes
      kCompleted_local = ++kCompleted_approx_[partitionIndex];
    }

  }  // enf of while loop

  // all k's are already covered. If all the nodes are completed notify and return, else just return.
  if (kCompleted_local >= N) {
    std::unique_lock<std::mutex> lock(kCompletedMutex_);
    kCompletedCondition_.notify_all();
    lock.unlock();

    // display
    if (BASE::ddpSettings_.debugPrintMT_) {
      BASE::printString("[MT]: [Thread " + std::to_string(threadId) + "], partition " + std::to_string(partitionIndex) + ", k " +
                        std::to_string(k) + ", kCompleted_local " + std::to_string(kCompleted_local) + ", KMax " + std::to_string(N) +
                        ": leaving executeApproximatePartitionLQWorker AND NOTIFYING ");
    }

  } else {
    // display
    if (BASE::ddpSettings_.debugPrintMT_) {
      BASE::printString("[MT]: [Thread " + std::to_string(threadId) + "], partition " + std::to_string(partitionIndex) + ", k " +
                        std::to_string(k) + ", kCompleted_local " + std::to_string(kCompleted_local) + ", KMax " + std::to_string(N) +
                        ": leaving executeApproximatePartitionLQWorker but NOT notifying ");
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_MP<STATE_DIM, INPUT_DIM>::calculatePartitionController(size_t partitionIndex) {
  {
    std::lock_guard<std::mutex> waitLock(workerWakeUpMutex_);
    subsystemProcessed_ = partitionIndex;
  }

  size_t N = BASE::SsTimeTrajectoryStock_[partitionIndex].size();

  if (N > 0) {
    // display
    if (BASE::ddpSettings_.debugPrintMT_) {
      std::cerr << "[MT]: Waking up workers to calculate controller for partition " + std::to_string(partitionIndex) << std::endl;
    }

    kTaken_ctrl_[partitionIndex] = 0;
    kCompleted_ctrl_[partitionIndex] = 0;

    {
      std::lock_guard<std::mutex> waitLock(workerWakeUpMutex_);
      workerTask_ = CALCULATE_CONTROLLER;
    }
    workerWakeUpCondition_.notify_all();

    // display
    if (BASE::ddpSettings_.debugPrintMT_) {
      BASE::printString("[MT]: Will wait now controllers have been calculated for partition " + std::to_string(partitionIndex));
    }

    // wait until all threads finish their task
    std::unique_lock<std::mutex> waitLock(kCompletedMutex_);
    while (kCompleted_ctrl_[partitionIndex].load() < N) {
      kCompletedCondition_.wait(waitLock);
    }
    waitLock.unlock();

    // reset threads to no task mode
    {
      std::lock_guard<std::mutex> waitLock(workerWakeUpMutex_);
      workerTask_ = IDLE;
    }

    // display
    if (BASE::ddpSettings_.debugPrintMT_) {
      std::cerr << "[MT]: Back to main thread, workers should now have designed controllers for partition " + std::to_string(partitionIndex)
                << std::endl;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_MP<STATE_DIM, INPUT_DIM>::executeCalculatePartitionController(size_t threadId, size_t partitionIndex) {
  size_t kCompleted_local = 0;
  int N = BASE::SsTimeTrajectoryStock_[partitionIndex].size();
  int k = -1;  // to make use that the while loop runs at least once

  while (k + 1 < N) {
    // update k
    k = kTaken_ctrl_[partitionIndex]++;

    if (k < N) {
      // display
      if (BASE::ddpSettings_.debugPrintMT_ && k % 10 == 0) {
        BASE::printString("[MT]: [Thread " + std::to_string(threadId) +
                          "]: Start calculating controller on index k = " + std::to_string(k) + " out of " + std::to_string(N - 1));
      }

      // execute calculateControllerWorker for the given partition and time node index (k)
      BASE::calculateControllerWorker(threadId, partitionIndex, k);

      // increment the number of completed nodes
      kCompleted_local = ++kCompleted_ctrl_[partitionIndex];
    }

  }  // enf of while loop

  // all k's are already covered. If all the nodes are completed notify and return, else just return.
  if (kCompleted_local >= N) {
    std::unique_lock<std::mutex> lock(kCompletedMutex_);
    kCompletedCondition_.notify_all();
    lock.unlock();

    // display
    if (BASE::ddpSettings_.debugPrintMT_) {
      BASE::printString("[MT]: [Thread " + std::to_string(threadId) + "], subsystem " + std::to_string(partitionIndex) +
                        ": leaving executeCalculatePartitionController() AND NOTIFYING ");
    }

  } else {
    // display
    if (BASE::ddpSettings_.debugPrintMT_) {
      BASE::printString("[MT]: [Thread " + std::to_string(threadId) + "], subsystem " + std::to_string(partitionIndex) +
                        ": leaving executeCalculatePartitionController() but NOT notifying ");
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_MP<STATE_DIM, INPUT_DIM>::executeLineSearchWorker(size_t threadId) {
  if (BASE::ddpSettings_.debugPrintMT_) {
    BASE::printString("[MT]: [Thread " + std::to_string(threadId) + "]: Starting executeLineSearchWorker. ");
  }

  // local search forward simulation's variables
  scalar_t lsTotalCost;
  scalar_t lsConstraint1ISE, lsConstraint2ISE, lsInequalityConstraintPenalty, lsInequalityConstraintISE;
  scalar_t lsConstraint1MaxNorm, lsConstraint2MaxNorm;
  linear_controller_array_t lsControllersStock(BASE::numPartitions_);
  scalar_array2_t lsTimeTrajectoriesStock(BASE::numPartitions_);
  size_array2_t lsEventsPastTheEndIndecesStock(BASE::numPartitions_);
  state_vector_array2_t lsStateTrajectoriesStock(BASE::numPartitions_);
  input_vector_array2_t lsInputTrajectoriesStock(BASE::numPartitions_);

  while (true) {
    size_t alphaExp = alphaTaken_++;
    scalar_t learningRate = BASE::maxLearningRate_ * std::pow(BASE::ddpSettings_.lineSearchContractionRate_, alphaExp);

    // break condition
    if (!numerics::almost_ge(learningRate, BASE::ddpSettings_.minLearningRate_) || alphaBestFound_.load()) {
      // display
      if (BASE::ddpSettings_.debugPrintMT_) {
        if (alphaBestFound_.load()) {
          BASE::printString("[MT]: [Thread " + std::to_string(threadId) +
                            "]: Leaving executeLineSearchWorker because best alpha is found OR no improvement for any alpha");
        } else {
          BASE::printString("[MT]: [Thread " + std::to_string(threadId) +
                            "]: Leaving executeLineSearchWorker because learningRate is less than BASE::ddpSettings_.minLearningRate_");
        }
      }

      break;
    }

    // display
    if (BASE::ddpSettings_.debugPrintMT_) {
      BASE::printString("[MT]: [Thread " + std::to_string(threadId) + "]: Trying learningRate " + std::to_string(learningRate));
    }

    // do a line search
    lsControllersStock = BASE::initLScontrollersStock_;
    BASE::lineSearchWorker(threadId, learningRate, lsTotalCost, lsConstraint1ISE, lsConstraint1MaxNorm, lsConstraint2ISE,
                           lsConstraint2MaxNorm, lsInequalityConstraintPenalty, lsInequalityConstraintISE, lsControllersStock,
                           lsTimeTrajectoriesStock, lsEventsPastTheEndIndecesStock, lsStateTrajectoriesStock, lsInputTrajectoriesStock);

    // break condition: make sure we do not alter an existing result
    if (alphaBestFound_.load()) {
      // display
      if (BASE::ddpSettings_.debugPrintMT_) {
        BASE::printString("[MT]: [Thread " + std::to_string(threadId) +
                          "]: Leaving executeLineSearchWorker because best alpha already found by another thread.");
      }

      break;
    }

    lineSearchResultMutex_.lock();

    // Based on the LS policy check whether the best solution should be updated with these results.
    bool updatePolicy = false;
    if (BASE::ddpSettings_.lsStepsizeGreedy_) {
      /*
       * Use stepsize greedy where cost should be better than the last iteration but learning rate
       * should be as high as possible. This is equivalent to a single core lineSearch.
       */

      if (lsTotalCost < (baselineTotalCost_ * (1 - 1e-3 * learningRate)) && learningRate > BASE::learningRateStar_) {
        updatePolicy = true;
        // display
        if (BASE::ddpSettings_.debugPrintMT_) {
          BASE::printString("[MT]: [LS, Thread " + std::to_string(threadId) + "]: stepsize-greedy mode: better stepsize and cost found: " +
                            std::to_string(lsTotalCost) + " at learningRate: " + std::to_string(learningRate));
        }
      } else {
        // display
        if (BASE::ddpSettings_.debugPrintMT_) {
          BASE::printString("[MT]: [LS, Thread " + std::to_string(threadId) +
                            "]: stepsize-greedy mode: no better combination found, cost " + std::to_string(lsTotalCost) +
                            " at learningRate: " + std::to_string(learningRate));
        }
      }

    } else {
      /*
       * line search acts cost greedy which minimize cost as much as possible
       */

      if (lsTotalCost < (BASE::nominalTotalCost_ * (1 - 1e-3 * learningRate))) {
        updatePolicy = true;
        if (BASE::ddpSettings_.debugPrintMT_) {
          BASE::printString("[MT]: [LS, Thread " + std::to_string(threadId) + "]: cost-greedy mode : better cost found: " +
                            std::to_string(lsTotalCost) + " at learningRate: " + std::to_string(learningRate));
        }
      } else {
        if (BASE::ddpSettings_.debugPrintMT_) {
          BASE::printString("[MT]: [LS, Thread " + std::to_string(threadId) + "]: cost-greedy mode : no better cost found, cost " +
                            std::to_string(lsTotalCost) + " at learningRate: " + std::to_string(learningRate) + ". Best cost was " +
                            std::to_string(BASE::nominalTotalCost_));
        }
      }
    }

    if (updatePolicy) {
      alphaExpBest_ = alphaExp;
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
      BASE::nominalEventsPastTheEndIndecesStock_.swap(lsEventsPastTheEndIndecesStock);
      BASE::nominalStateTrajectoriesStock_.swap(lsStateTrajectoriesStock);
      BASE::nominalInputTrajectoriesStock_.swap(lsInputTrajectoriesStock);
    }

    alphaProcessed_[alphaExp] = true;

    // we now check if all alphas prior to the best have been processed, this also covers the case that there is no better alpha
    bool allPreviousAlphasProcessed = true;
    for (size_t i = 0; i < alphaExpBest_; i++) {
      if (!alphaProcessed_[i]) {
        allPreviousAlphasProcessed = false;
        break;
      }
    }

    if (allPreviousAlphasProcessed) {
      alphaBestFound_ = true;
      event_handler_t::activateKillIntegration();  // kill all integrators
      if (BASE::ddpSettings_.displayInfo_) {
        BASE::printString("\t LS: terminate other rollouts with different alphas. alpha_best found or terminating without improvement.");
      }
    }

    lineSearchResultMutex_.unlock();

  }  // end of while loop

  {
    std::lock_guard<std::mutex> waitLock(alphaBestFoundMutex_);
    // add to the number of threads that finished their tasks
    lsWorkerCompleted_++;

    if (BASE::ddpSettings_.debugPrintMT_) {
      BASE::printString("[MT]: [Thread " + std::to_string(threadId) + "]: Leaving executeLineSearchWorker ");
    }

    if (lsWorkerCompleted_ >= BASE::ddpSettings_.nThreads_) {
      alphaBestFoundCondition_.notify_all();

      if (BASE::ddpSettings_.debugPrintMT_) {
        BASE::printString("[MT]: NOTIFYING LS WORKERs since all workers are now done.");
      }
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
typename SLQ_MP<STATE_DIM, INPUT_DIM>::scalar_t SLQ_MP<STATE_DIM, INPUT_DIM>::solveSequentialRiccatiEquations(
    const state_matrix_t& SmFinal, const state_vector_t& SvFinal, const eigen_scalar_t& sFinal) {
  numSubsystemsProcessed_ = 0;

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
    if (BASE::ddpSettings_.debugPrintMT_) {
      BASE::printString("[MT]: Waking up workers to do RiccatiSolver Task.");
    }

    {
      std::lock_guard<std::mutex> waitLock(workerWakeUpMutex_);
      workerTask_ = SOLVE_RICCATI;
    }
    workerWakeUpCondition_.notify_all();

    if (BASE::ddpSettings_.debugPrintMT_) {
      BASE::printString("[MT]: Will wait now until workers have done RiccatiSolver Task.");
    }

    std::unique_lock<std::mutex> waitLock(riccatiSolverBarrierMutex_);
    while (numSubsystemsProcessed_.load() < BASE::numPartitions_ && !workerException_) {
      riccatiSolverCompletedCondition_.wait(waitLock);
    }
    waitLock.unlock();

    {
      std::lock_guard<std::mutex> waitLock(workerWakeUpMutex_);
      workerTask_ = IDLE;
    }
  }

  if (BASE::ddpSettings_.debugPrintMT_) {
    BASE::printString("[MT]: Iteration: " + std::to_string(BASE::iteration_) + " done.");
    BASE::printString("----------------------------------");
    BASE::printString("----------------------------------");
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
void SLQ_MP<STATE_DIM, INPUT_DIM>::executeRiccatiSolver(size_t threadId) {
  for (int i = endingIndicesRiccatiWorker_[threadId]; i >= startingIndicesRiccatiWorker_[threadId]; i--) {
    if (BASE::ddpSettings_.debugPrintMT_) {
      BASE::printString("[MT]: Thread " + std::to_string(threadId) + " processing subsystem " + std::to_string(i));
    }

    // for inactive subsystems
    if (i < BASE::initActivePartition_ || i > BASE::finalActivePartition_) {
      BASE::SsTimeTrajectoryStock_[i].clear();
      BASE::SmTrajectoryStock_[i].clear();
      BASE::SvTrajectoryStock_[i].clear();
      BASE::SveTrajectoryStock_[i].clear();
      BASE::sTrajectoryStock_[i].clear();

      // lock data
      std::unique_lock<std::mutex> dataWriteLock(riccatiSolverDataMutex_);

      BASE::SmFinalStock_[i].setZero();
      BASE::SvFinalStock_[i].setZero();
      BASE::SveFinalStock_[i].setZero();
      BASE::sFinalStock_[i].setZero();
      BASE::xFinalStock_[i].setZero();

      // unlock data
      dataWriteLock.unlock();

      numSubsystemsProcessed_++;

      continue;
    }

    // lock data
    std::unique_lock<std::mutex> dataReadLock(riccatiSolverDataMutex_);

    state_matrix_t SmFinal = BASE::SmFinalStock_[i];
    state_vector_t SvFinal = BASE::SvFinalStock_[i];
    state_vector_t SveFinal = BASE::SveFinalStock_[i];
    eigen_scalar_t sFinal = BASE::sFinalStock_[i];
    state_vector_t xFinal = BASE::xFinalStock_[i];

    // unlock data
    dataReadLock.unlock();

    // modify the end subsystem final values based on the cached values for asynchronous run
    if (i == endingIndicesRiccatiWorker_[threadId] && i < BASE::finalActivePartition_) {
      const state_vector_t& x = BASE::nominalStateTrajectoriesStock_[i + 1].front();
      SvFinal += SmFinal * (x - xFinal);
      // TODO for sFinal
    }

    // solve the backward pass
    BASE::solveSlqRiccatiEquationsWorker(threadId, i, SmFinal, SvFinal, sFinal, SveFinal);

    // lock data
    std::unique_lock<std::mutex> dataWriteLock(riccatiSolverDataMutex_);

    // set the final value for next Riccati equation
    if (i > BASE::initActivePartition_) {
      BASE::SmFinalStock_[i - 1] = BASE::SmTrajectoryStock_[i].front();
      BASE::SvFinalStock_[i - 1] = BASE::SvTrajectoryStock_[i].front();
      BASE::SveFinalStock_[i - 1] = BASE::SveTrajectoryStock_[i].front();
      BASE::sFinalStock_[i - 1] = BASE::sTrajectoryStock_[i].front();
      BASE::xFinalStock_[i - 1] = BASE::nominalStateTrajectoriesStock_[i].front();
    }

    // unlock data
    dataWriteLock.unlock();

    numSubsystemsProcessed_++;
  }

  // notify the main thread so that it stops waiting
  //	std::unique_lock<std::mutex> lock(riccatiSolverBarrierNotifyMutex_);
  std::unique_lock<std::mutex> lock(riccatiSolverBarrierMutex_);
  riccatiSolverCompletedCondition_.notify_one();
  lock.unlock();
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_MP<STATE_DIM, INPUT_DIM>::distributeWork() {
  const int N = BASE::ddpSettings_.nThreads_;
  startingIndicesRiccatiWorker_.resize(N);
  endingIndicesRiccatiWorker_.resize(N);

  //	if (BASE::numPartitions_ < N)
  //		throw std::runtime_error("Number of threads is bigger than number of subsystems");

  int subsystemsPerThread = (BASE::finalActivePartition_ - BASE::initActivePartition_ + 1) / N;
  int remainingSubsystems = (BASE::finalActivePartition_ - BASE::initActivePartition_ + 1) % N;

  int startingId, endingId = BASE::finalActivePartition_;
  for (size_t i = 0; i < N; i++) {
    endingIndicesRiccatiWorker_[i] = endingId;
    if (remainingSubsystems > 0) {
      startingId = endingId - subsystemsPerThread;
      remainingSubsystems--;
    } else {
      startingId = endingId - subsystemsPerThread + 1;
    }
    startingIndicesRiccatiWorker_[i] = startingId;
    endingId = startingId - 1;
  }

  // adding the inactive subsystems
  endingIndicesRiccatiWorker_.front() = BASE::numPartitions_ - 1;
  startingIndicesRiccatiWorker_.back() = 0;

  if (BASE::ddpSettings_.displayInfo_) {
    std::cerr << "Initial Active Subsystem: " << BASE::initActivePartition_ << std::endl;
    std::cerr << "Final Active Subsystem:   " << BASE::finalActivePartition_ << std::endl;
    std::cerr << "Backward path work distribution:" << std::endl;
    for (size_t i = 0; i < N; i++) {
      std::cerr << "start: " << startingIndicesRiccatiWorker_[i] << "\t";
      std::cerr << "end: " << endingIndicesRiccatiWorker_[i] << "\t";
      std::cerr << "num: " << endingIndicesRiccatiWorker_[i] - startingIndicesRiccatiWorker_[i] + 1 << std::endl;
      ;
    }
    std::cerr << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_MP<STATE_DIM, INPUT_DIM>::runInit() {
  // disable Eigen multi-threading
  Eigen::setNbThreads(1);

  // distribute work
  distributeWork();

  // run BASE routine
  BASE::runInit();

  // restore default Eigen thread number
  Eigen::setNbThreads(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_MP<STATE_DIM, INPUT_DIM>::runIteration() {
  // disable Eigen multi-threading
  Eigen::setNbThreads(1);

  {
    std::lock_guard<std::mutex> lock(workerExceptionMutex_);
    workerException_ = nullptr;
  }

  // run BASE routine
  BASE::runIteration();

  // restore default Eigen thread number
  Eigen::setNbThreads(0);

  {
    std::lock_guard<std::mutex> lock(workerExceptionMutex_);
    if (workerException_) {
      std::rethrow_exception(workerException_);
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ_MP<STATE_DIM, INPUT_DIM>::runExit() {
  // disable Eigen multi-threading
  Eigen::setNbThreads(1);

  // run BASE routine
  BASE::runExit();

  // restore default Eigen thread number
  Eigen::setNbThreads(0);
}

}  // namespace ocs2
