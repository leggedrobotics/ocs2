#pragma once

#include <ocs2_core/thread_support/ThreadPool.h>

#include "ocs2_mpcnet/rollout/MpcnetRolloutManager.h"

namespace ocs2 {

/**
 *  Base class for all MPC-Net interfaces between C++ and Python.
 */
class MpcnetInterfaceBase {
 public:
  using data_point_t = MpcnetRolloutManager::data_point_t;
  using data_array_t = MpcnetRolloutManager::data_array_t;
  using data_ptr_t = MpcnetRolloutManager::data_ptr_t;
  using metrics_t = MpcnetRolloutManager::metrics_t;
  using metrics_array_t = MpcnetRolloutManager::metrics_array_t;
  using metrics_ptr_t = MpcnetRolloutManager::metrics_ptr_t;

 public:
  /**
   * Default destructor.
   */
  virtual ~MpcnetInterfaceBase() = default;

  /**
   * @see MpcnetRolloutManager::startDataGeneration()
   */
  void startDataGeneration(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep, size_t dataDecimation, size_t nSamples,
                           const matrix_t& samplingCovariance, const std::vector<SystemObservation>& initialObservations,
                           const std::vector<ModeSchedule>& modeSchedules, const std::vector<TargetTrajectories>& targetTrajectories);

  /**
   * @see MpcnetRolloutManager::isDataGenerationDone()
   */
  bool isDataGenerationDone();

  /**
   * @see MpcnetRolloutManager::getGeneratedData()
   */
  data_array_t getGeneratedData();

  /**
   * @see MpcnetRolloutManager::startPolicyEvaluation()
   */
  void startPolicyEvaluation(const std::string& policyFilePath, scalar_t timeStep,
                             const std::vector<SystemObservation>& initialObservations, const std::vector<ModeSchedule>& modeSchedules,
                             const std::vector<TargetTrajectories>& targetTrajectories);

  /**
   * @see MpcnetRolloutManager::isPolicyEvaluationDone()
   */
  bool isPolicyEvaluationDone();

  /**
   * @see MpcnetRolloutManager::getComputedMetrics()
   */
  metrics_array_t getComputedMetrics();

 protected:
  /**
   * Default constructor.
   */
  MpcnetInterfaceBase() = default;

  std::unique_ptr<MpcnetRolloutManager> mpcnetRolloutManagerPtr_;
};

}  // namespace ocs2
