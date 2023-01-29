/******************************************************************************
Copyright (c) 2022, Farbod Farshidian. All rights reserved.

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

#include "ocs2_mpcnet_core/rollout/MpcnetRolloutManager.h"

namespace ocs2 {
namespace mpcnet {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MpcnetRolloutManager::MpcnetRolloutManager(size_t nDataGenerationThreads, size_t nPolicyEvaluationThreads,
                                           std::vector<std::unique_ptr<MPC_BASE>> mpcPtrs,
                                           std::vector<std::unique_ptr<MpcnetControllerBase>> mpcnetPtrs,
                                           std::vector<std::unique_ptr<RolloutBase>> rolloutPtrs,
                                           std::vector<std::shared_ptr<MpcnetDefinitionBase>> mpcnetDefinitionPtrs,
                                           std::vector<std::shared_ptr<ReferenceManagerInterface>> referenceManagerPtrs) {
  // data generation
  nDataGenerationThreads_ = nDataGenerationThreads;
  if (nDataGenerationThreads_ > 0) {
    dataGenerationThreadPoolPtr_.reset(new ThreadPool(nDataGenerationThreads_));
    dataGenerationPtrs_.reserve(nDataGenerationThreads);
    for (int i = 0; i < nDataGenerationThreads; i++) {
      dataGenerationPtrs_.push_back(
          std::make_unique<MpcnetDataGeneration>(std::move(mpcPtrs.at(i)), std::move(mpcnetPtrs.at(i)), std::move(rolloutPtrs.at(i)),
                                                 std::move(mpcnetDefinitionPtrs.at(i)), referenceManagerPtrs.at(i)));
    }
  }

  // policy evaluation
  nPolicyEvaluationThreads_ = nPolicyEvaluationThreads;
  if (nPolicyEvaluationThreads_ > 0) {
    policyEvaluationThreadPoolPtr_.reset(new ThreadPool(nPolicyEvaluationThreads_));
    policyEvaluationPtrs_.reserve(nPolicyEvaluationThreads_);
    for (int i = nDataGenerationThreads_; i < (nDataGenerationThreads_ + nPolicyEvaluationThreads_); i++) {
      policyEvaluationPtrs_.push_back(
          std::make_unique<MpcnetPolicyEvaluation>(std::move(mpcPtrs.at(i)), std::move(mpcnetPtrs.at(i)), std::move(rolloutPtrs.at(i)),
                                                   std::move(mpcnetDefinitionPtrs.at(i)), referenceManagerPtrs.at(i)));
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MpcnetRolloutManager::startDataGeneration(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep, size_t dataDecimation,
                                               size_t nSamples, const matrix_t& samplingCovariance,
                                               const std::vector<SystemObservation>& initialObservations,
                                               const std::vector<ModeSchedule>& modeSchedules,
                                               const std::vector<TargetTrajectories>& targetTrajectories) {
  if (nDataGenerationThreads_ <= 0) {
    throw std::runtime_error("[MpcnetRolloutManager::startDataGeneration] cannot work without at least one data generation thread.");
  }

  // reset variables
  dataGenerationFtrs_.clear();
  nDataGenerationTasksDone_ = 0;

  // push tasks into pool
  for (int i = 0; i < initialObservations.size(); i++) {
    dataGenerationFtrs_.push_back(dataGenerationThreadPoolPtr_->run([=](int threadNumber) {
      const auto* result =
          dataGenerationPtrs_[threadNumber]->run(alpha, policyFilePath, timeStep, dataDecimation, nSamples, samplingCovariance,
                                                 initialObservations.at(i), modeSchedules.at(i), targetTrajectories.at(i));
      nDataGenerationTasksDone_++;
      // print thread and task number
      std::cerr << "Data generation thread " << threadNumber << " finished task " << nDataGenerationTasksDone_ << "\n";
      return result;
    }));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MpcnetRolloutManager::isDataGenerationDone() {
  if (nDataGenerationThreads_ <= 0) {
    throw std::runtime_error("[MpcnetRolloutManager::isDataGenerationDone] cannot work without at least one data generation thread.");
  }
  if (dataGenerationFtrs_.size() <= 0) {
    throw std::runtime_error(
        "[MpcnetRolloutManager::isDataGenerationDone] cannot return if startDataGeneration has not been triggered once.");
  }

  // check if done
  if (nDataGenerationTasksDone_ < dataGenerationFtrs_.size()) {
    return false;
  } else if (nDataGenerationTasksDone_ == dataGenerationFtrs_.size()) {
    return true;
  } else {
    throw std::runtime_error("[MpcnetRolloutManager::isDataGenerationDone] error since more tasks done than futures available.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const data_array_t& MpcnetRolloutManager::getGeneratedData() {
  if (nDataGenerationThreads_ <= 0) {
    throw std::runtime_error("[MpcnetRolloutManager::getGeneratedData] cannot work without at least one data generation thread.");
  }
  if (!isDataGenerationDone()) {
    throw std::runtime_error("[MpcnetRolloutManager::getGeneratedData] cannot get data when data generation is not done.");
  }

  // clear data array
  dataArray_.clear();

  // get pointers to data
  std::vector<const data_array_t*> dataPtrs;
  dataPtrs.reserve(dataGenerationFtrs_.size());
  for (auto& dataGenerationFtr : dataGenerationFtrs_) {
    try {
      // get results from futures of the tasks
      dataPtrs.push_back(dataGenerationFtr.get());
    } catch (const std::exception& e) {
      // print error for exceptions
      std::cerr << "[MpcnetRolloutManager::getGeneratedData] a standard exception was caught, with message: " << e.what() << "\n";
    }
  }

  // find number of data points
  int nDataPoints = 0;
  for (int i = 0; i < dataPtrs.size(); i++) {
    nDataPoints += dataPtrs[i]->size();
  }

  // fill data array
  dataArray_.reserve(nDataPoints);
  for (const auto dataPtr : dataPtrs) {
    dataArray_.insert(dataArray_.end(), dataPtr->begin(), dataPtr->end());
  }

  // return data array
  return dataArray_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MpcnetRolloutManager::startPolicyEvaluation(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep,
                                                 const std::vector<SystemObservation>& initialObservations,
                                                 const std::vector<ModeSchedule>& modeSchedules,
                                                 const std::vector<TargetTrajectories>& targetTrajectories) {
  if (nPolicyEvaluationThreads_ <= 0) {
    throw std::runtime_error("[MpcnetRolloutManager::startPolicyEvaluation] cannot work without at least one policy evaluation thread.");
  }

  // reset variables
  policyEvaluationFtrs_.clear();
  nPolicyEvaluationTasksDone_ = 0;

  // push tasks into pool
  for (int i = 0; i < initialObservations.size(); i++) {
    policyEvaluationFtrs_.push_back(policyEvaluationThreadPoolPtr_->run([=](int threadNumber) {
      const auto result = policyEvaluationPtrs_[threadNumber]->run(alpha, policyFilePath, timeStep, initialObservations.at(i),
                                                                   modeSchedules.at(i), targetTrajectories.at(i));
      nPolicyEvaluationTasksDone_++;
      // print thread and task number
      std::cerr << "Policy evaluation thread " << threadNumber << " finished task " << nPolicyEvaluationTasksDone_ << "\n";
      return result;
    }));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MpcnetRolloutManager::isPolicyEvaluationDone() {
  if (nPolicyEvaluationThreads_ <= 0) {
    throw std::runtime_error("[MpcnetRolloutManager::isPolicyEvaluationDone] cannot work without at least one policy evaluation thread.");
  }
  if (policyEvaluationFtrs_.size() <= 0) {
    throw std::runtime_error(
        "[MpcnetRolloutManager::isPolicyEvaluationDone] cannot return if startPolicyEvaluation has not been triggered once.");
  }

  // check if done
  if (nPolicyEvaluationTasksDone_ < policyEvaluationFtrs_.size()) {
    return false;
  } else if (nPolicyEvaluationTasksDone_ == policyEvaluationFtrs_.size()) {
    return true;
  } else {
    throw std::runtime_error("[MpcnetRolloutManager::isPolicyEvaluationDone] error since more tasks done than futures available.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
metrics_array_t MpcnetRolloutManager::getComputedMetrics() {
  if (nPolicyEvaluationThreads_ <= 0) {
    throw std::runtime_error("[MpcnetRolloutManager::getComputedMetrics] cannot work without at least one policy evaluation thread.");
  }
  if (!isPolicyEvaluationDone()) {
    throw std::runtime_error("[MpcnetRolloutManager::getComputedMetrics] cannot get metrics when policy evaluation is not done.");
  }

  // get metrics and fill metrics array
  metrics_array_t metricsArray;
  metricsArray.reserve(policyEvaluationFtrs_.size());
  for (auto& policyEvaluationFtr : policyEvaluationFtrs_) {
    try {
      // get results from futures of the tasks
      metricsArray.push_back(policyEvaluationFtr.get());
    } catch (const std::exception& e) {
      // print error for exceptions
      std::cerr << "[MpcnetRolloutManager::getComputedMetrics] a standard exception was caught, with message: " << e.what() << "\n";
    }
  }

  // return metrics array
  return metricsArray;
}

}  // namespace mpcnet
}  // namespace ocs2
