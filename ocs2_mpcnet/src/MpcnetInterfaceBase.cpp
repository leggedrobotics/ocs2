#include "ocs2_mpcnet/MpcnetInterfaceBase.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MpcnetInterfaceBase::startDataGeneration(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep, size_t dataDecimation,
                                              size_t nSamples, const matrix_t& samplingCovariance,
                                              const std::vector<SystemObservation>& initialObservations,
                                              const std::vector<ModeSchedule>& modeSchedules,
                                              const std::vector<TargetTrajectories>& targetTrajectories) {
  mpcnetRolloutManagerPtr_->startDataGeneration(alpha, policyFilePath, timeStep, dataDecimation, nSamples, samplingCovariance,
                                                initialObservations, modeSchedules, targetTrajectories);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MpcnetInterfaceBase::isDataGenerationDone() {
  return mpcnetRolloutManagerPtr_->isDataGenerationDone();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MpcnetInterfaceBase::data_array_t MpcnetInterfaceBase::getGeneratedData() {
  return mpcnetRolloutManagerPtr_->getGeneratedData();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MpcnetInterfaceBase::startPolicyEvaluation(const std::string& policyFilePath, scalar_t timeStep,
                                                const std::vector<SystemObservation>& initialObservations,
                                                const std::vector<ModeSchedule>& modeSchedules,
                                                const std::vector<TargetTrajectories>& targetTrajectories) {
  mpcnetRolloutManagerPtr_->startPolicyEvaluation(policyFilePath, timeStep, initialObservations, modeSchedules, targetTrajectories);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MpcnetInterfaceBase::isPolicyEvaluationDone() {
  return mpcnetRolloutManagerPtr_->isPolicyEvaluationDone();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MpcnetInterfaceBase::metrics_array_t MpcnetInterfaceBase::getComputedMetrics() {
  return mpcnetRolloutManagerPtr_->getComputedMetrics();
}

}  // namespace ocs2
