#include "ocs2_mpcnet/rollout/MpcnetRolloutManager.h"

namespace ocs2 {

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
      dataGenerationPtrs_.push_back(std::unique_ptr<MpcnetDataGeneration>(
          new MpcnetDataGeneration(std::move(mpcPtrs.at(i)), std::move(mpcnetPtrs.at(i)), std::move(rolloutPtrs.at(i)),
                                   std::move(mpcnetDefinitionPtrs.at(i)), referenceManagerPtrs.at(i))));
    }
  }

  // policy evaluation
  nPolicyEvaluationThreads_ = nPolicyEvaluationThreads;
  if (nPolicyEvaluationThreads_ > 0) {
    policyEvaluationThreadPoolPtr_.reset(new ThreadPool(nPolicyEvaluationThreads_));
    policyEvaluationPtrs_.reserve(nPolicyEvaluationThreads_);
    for (int i = nDataGenerationThreads_; i < (nDataGenerationThreads_ + nPolicyEvaluationThreads_); i++) {
      policyEvaluationPtrs_.push_back(std::unique_ptr<MpcnetPolicyEvaluation>(
          new MpcnetPolicyEvaluation(std::move(mpcPtrs.at(i)), std::move(mpcnetPtrs.at(i)), std::move(rolloutPtrs.at(i)),
                                     std::move(mpcnetDefinitionPtrs.at(i)), referenceManagerPtrs.at(i))));
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
      data_ptr_t result;
      result = dataGenerationPtrs_[threadNumber]->run(alpha, policyFilePath, timeStep, dataDecimation, nSamples, samplingCovariance,
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
MpcnetRolloutManager::data_array_t MpcnetRolloutManager::getGeneratedData() {
  if (nDataGenerationThreads_ <= 0) {
    throw std::runtime_error("[MpcnetRolloutManager::getGeneratedData] cannot work without at least one data generation thread.");
  }
  if (!isDataGenerationDone()) {
    throw std::runtime_error("[MpcnetRolloutManager::getGeneratedData] cannot get data when data generation is not done.");
  }

  // get pointers to data
  std::vector<data_ptr_t> dataPtrs;
  for (int i = 0; i < dataGenerationFtrs_.size(); i++) {
    try {
      // get results from futures of the tasks
      dataPtrs.push_back(dataGenerationFtrs_[i].get());
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
  data_array_t dataArray;
  dataArray.reserve(nDataPoints);
  for (int i = 0; i < dataPtrs.size(); i++) {
    for (int j = 0; j < dataPtrs[i]->size(); j++) {
      dataArray.push_back((*dataPtrs[i])[j]);
    }
  }

  // return data
  return dataArray;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MpcnetRolloutManager::startPolicyEvaluation(const std::string& policyFilePath, scalar_t timeStep,
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
      metrics_ptr_t result;
      result = policyEvaluationPtrs_[threadNumber]->run(policyFilePath, timeStep, initialObservations.at(i), modeSchedules.at(i),
                                                        targetTrajectories.at(i));
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
MpcnetRolloutManager::metrics_array_t MpcnetRolloutManager::getComputedMetrics() {
  if (nPolicyEvaluationThreads_ <= 0) {
    throw std::runtime_error("[MpcnetRolloutManager::getComputedMetrics] cannot work without at least one policy evaluation thread.");
  }
  if (!isPolicyEvaluationDone()) {
    throw std::runtime_error("[MpcnetRolloutManager::getComputedMetrics] cannot get metrics when policy evaluation is not done.");
  }

  // get pointers to metrics
  std::vector<metrics_ptr_t> metricsPtrs;
  for (int i = 0; i < policyEvaluationFtrs_.size(); i++) {
    try {
      // get results from futures of the tasks
      metricsPtrs.push_back(policyEvaluationFtrs_[i].get());
    } catch (const std::exception& e) {
      // print error for exceptions
      std::cerr << "[MpcnetRolloutManager::getComputedMetrics] a standard exception was caught, with message: " << e.what() << "\n";
    }
  }

  // fill metrics array
  metrics_array_t metricsArray;
  metricsArray.reserve(metricsPtrs.size());
  for (int i = 0; i < metricsPtrs.size(); i++) {
    metricsArray.push_back((*metricsPtrs[i]));
  }

  // return metrics
  return metricsArray;
}

}  // namespace ocs2
