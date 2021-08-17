#pragma once

#include <memory>

#include <ocs2_core/control/ControllerBase.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerInterface.h>

#include "ocs2_mpcnet/MpcnetDefinitionBase.h"

namespace ocs2 {

/**
 * The base class for all controllers that use a MPC-Net policy.
 */
class MpcnetControllerBase : public ControllerBase {
 public:
  using Base = ControllerBase;

  /**
   * Constructor.
   * @param [in] mpcnetDefinitionPtr : Pointer to the MPC-Net definitions.
   * @param [in] referenceManagerPtr : Pointer to the reference manager.
   */
  MpcnetControllerBase(std::shared_ptr<MpcnetDefinitionBase> mpcnetDefinitionPtr,
                       std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr)
      : mpcnetDefinitionPtr_(mpcnetDefinitionPtr), referenceManagerPtr_(referenceManagerPtr) {}

  /**
   * Default destructor.
   */
  ~MpcnetControllerBase() override = default;

  /**
   * Load the model of the policy.
   * @param [in] policyFilePath : Path to the file with the model of the policy.
   */
  virtual void loadPolicyModel(const std::string& policyFilePath) = 0;

  /**
   * Get the generalized time.
   * @param [in] t : Absolute time.
   * @return Generalized time.
   */
  vector_t getGeneralizedTime(scalar_t t) { return mpcnetDefinitionPtr_->getGeneralizedTime(t, referenceManagerPtr_->getModeSchedule()); }

  /**
   * Get the relative state.
   * @param [in] t : Absolute time.
   * @param [in] x : Robot state.
   * @return Relative state.
   */
  vector_t getRelativeState(scalar_t t, const vector_t& x) {
    return mpcnetDefinitionPtr_->getRelativeState(t, x, referenceManagerPtr_->getTargetTrajectories());
  }

  ControllerType getType() const override { return ControllerType::MPCNET; }

  MpcnetControllerBase* clone() const override = 0;

 protected:
  std::shared_ptr<MpcnetDefinitionBase> mpcnetDefinitionPtr_;
  std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr_;
};

}  // namespace ocs2
