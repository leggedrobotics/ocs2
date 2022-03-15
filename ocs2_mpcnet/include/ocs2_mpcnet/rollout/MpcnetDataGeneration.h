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
 *  A class for generating MPC data from a system that is forward simulated with a behavioral controller.
 *  The behavioral policy is a mixture of an MPC policy and an MPC-Net policy (e.g. a neural network).
 */
class MpcnetDataGeneration {
 public:
  struct DataPoint {
    size_t mode;
    scalar_t t;
    vector_t x;
    vector_t u;
    vector_t generalizedTime;
    vector_t relativeState;
    matrix_t inputTransformation;
    ScalarFunctionQuadraticApproximation hamiltonian;
  };
  using DataArray = std::vector<DataPoint>;
  using DataPtr = std::unique_ptr<DataArray>;

  /**
   * Constructor.
   * @param [in] mpcPtr : Pointer to the MPC solver to be used (this class takes ownership).
   * @param [in] mpcnetPtr : Pointer to the MPC-Net policy to be used (this class takes ownership).
   * @param [in] rolloutPtr : Pointer to the rollout to be used (this class takes ownership).
   * @param [in] mpcnetDefinitionPtr : Pointer to the MPC-Net definitions to be used (shared ownership).
   * @param [in] referenceManagerPtr : Pointer to the reference manager to be used (shared ownership).
   */
  MpcnetDataGeneration(std::unique_ptr<MPC_BASE> mpcPtr, std::unique_ptr<MpcnetControllerBase> mpcnetPtr,
                       std::unique_ptr<RolloutBase> rolloutPtr, std::shared_ptr<MpcnetDefinitionBase> mpcnetDefinitionPtr,
                       std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr)
      : mpcPtr_(std::move(mpcPtr)),
        mpcnetPtr_(std::move(mpcnetPtr)),
        rolloutPtr_(std::move(rolloutPtr)),
        mpcnetDefinitionPtr_(mpcnetDefinitionPtr),
        referenceManagerPtr_(referenceManagerPtr) {}

  /**
   * Default destructor.
   */
  virtual ~MpcnetDataGeneration() = default;

  /**
   * Deleted copy constructor.
   */
  MpcnetDataGeneration(const MpcnetDataGeneration&) = delete;

  /**
   * Deleted copy assignment.
   */
  MpcnetDataGeneration& operator=(const MpcnetDataGeneration&) = delete;

  /**
   * Run the data generation.
   * @param [in] alpha : The mixture parameter for the behavioral controller.
   * @param [in] policyFilePath : The path to the file with the learned policy for the behavioral controller.
   * @param [in] timeStep : The time step for the forward simulation of the system with the behavioral controller.
   * @param [in] dataDecimation : The integer factor used for downsampling the data signal.
   * @param [in] nSamples : The number of samples drawn from a multivariate normal distribution around the nominal states.
   * @param [in] samplingCovariance : The covariance matrix used for sampling from a multivariate normal distribution.
   * @param [in] initialObservation : The initial system observation to start from (time and state required).
   * @param [in] modeSchedule : The mode schedule providing the event times and mode sequence.
   * @param [in] targetTrajectories : The target trajectories to be tracked.
   * @return Pointer to the generated data.
   */
  DataPtr run(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep, size_t dataDecimation, size_t nSamples,
              const matrix_t& samplingCovariance, const SystemObservation& initialObservation, const ModeSchedule& modeSchedule,
              const TargetTrajectories& targetTrajectories);

 private:
  std::unique_ptr<MPC_BASE> mpcPtr_;
  std::unique_ptr<MpcnetControllerBase> mpcnetPtr_;
  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::shared_ptr<MpcnetDefinitionBase> mpcnetDefinitionPtr_;
  std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr_;
};

}  // namespace ocs2
