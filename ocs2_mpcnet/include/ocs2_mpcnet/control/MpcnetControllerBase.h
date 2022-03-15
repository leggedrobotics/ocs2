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

  ControllerType getType() const override { return ControllerType::MPCNET; }

  MpcnetControllerBase* clone() const override = 0;

 protected:
  /**
   * Copy constructor.
   */
  MpcnetControllerBase(const MpcnetControllerBase& other) : MpcnetControllerBase(other.mpcnetDefinitionPtr_, other.referenceManagerPtr_) {}

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

  /**
   * Get the input transformation.
   * @param[in] t : Absolute time.
   * @param[in] x : Robot state.
   * @return The input transformation.
   */
  matrix_t getInputTransformation(scalar_t t, const vector_t& x) { return mpcnetDefinitionPtr_->getInputTransformation(t, x); }

  std::shared_ptr<MpcnetDefinitionBase> mpcnetDefinitionPtr_;
  std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr_;
};

}  // namespace ocs2
