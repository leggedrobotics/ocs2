#pragma once

#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/rollout/RolloutBase.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerInterface.h>

#include "ocs2_mpcnet/MpcnetDefinitionBase.h"
#include "ocs2_mpcnet/control/MpcnetControllerBase.h"

namespace ocs2 {

/**
 *  A class for evaluating a policy for a system that is forward simulated with a learned controller.
 */
class MpcnetPolicyEvaluation {
 public:
  using mpc_t = MPC_BASE;
  using mpcnet_t = MpcnetControllerBase;
  using rollout_t = RolloutBase;
  using mpcnet_definition_t = MpcnetDefinitionBase;
  using reference_manager_t = ReferenceManagerInterface;

  struct Metrics {
    scalar_t survivalTime = 0.0;
    scalar_t incurredHamiltonian = 0.0;
  };
  using MetricsArray = std::vector<Metrics>;
  using MetricsPtr = std::unique_ptr<Metrics>;

  /**
   * Constructor.
   * @param [in] mpcPtr: Pointer to the MPC solver to be used (this class takes ownership).
   * @param [in] mpcnetPtr: Pointer to the MPC-Net policy to be used (this class takes ownership).
   * @param [in] rolloutPtr: Pointer to the rollout to be used (this class takes ownership).
   * @param [in] mpcnetDefinitionPtr: Pointer to the MPC-Net definitions to be used (shared ownership).
   * @param [in] referenceManagerPtr: Pointer to the reference manager to be used (shared ownership).
   */
  MpcnetPolicyEvaluation(std::unique_ptr<mpc_t> mpcPtr, std::unique_ptr<mpcnet_t> mpcnetPtr, std::unique_ptr<rollout_t> rolloutPtr,
                         std::shared_ptr<mpcnet_definition_t> mpcnetDefinitionPtr, std::shared_ptr<reference_manager_t> referenceManagerPtr)
      : mpcPtr_(std::move(mpcPtr)),
        mpcnetPtr_(std::move(mpcnetPtr)),
        rolloutPtr_(std::move(rolloutPtr)),
        mpcnetDefinitionPtr_(mpcnetDefinitionPtr),
        referenceManagerPtr_(referenceManagerPtr) {}

  /**
   * Default destructor.
   */
  virtual ~MpcnetPolicyEvaluation() = default;

  /**
   * Run the policy evaluation.
   * @param [in] policyFilePath : The path to the file with the learned policy for the controller.
   * @param [in] timeStep : The time step for the forward simulation of the system with the controller.
   * @param [in] initialObservation : The initial system observation to start from (time and state required).
   * @param [in] modeSchedule : The mode schedule providing the event times and mode sequence.
   * @param [in] targetTrajectories : The target trajectories to be tracked.
   * @return Pointer to the computed metrics.
   */
  MetricsPtr run(const std::string& policyFilePath, scalar_t timeStep, const SystemObservation& initialObservation,
                 const ModeSchedule& modeSchedule, const TargetTrajectories& targetTrajectories);

 private:
  std::unique_ptr<mpc_t> mpcPtr_;
  std::unique_ptr<mpcnet_t> mpcnetPtr_;
  std::unique_ptr<rollout_t> rolloutPtr_;
  std::shared_ptr<mpcnet_definition_t> mpcnetDefinitionPtr_;
  std::shared_ptr<reference_manager_t> referenceManagerPtr_;
};

}  // namespace ocs2
