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

#include "ocs2_mpcnet_core/control/MpcnetControllerBase.h"

namespace ocs2 {
namespace mpcnet {

/**
 * A behavioral controller that computes the input based on a mixture of an optimal policy (e.g. implicitly found via MPC)
 * and a learned policy (e.g. explicitly represented by a neural network).
 * The behavioral policy is pi_behavioral = alpha * pi_optimal + (1 - alpha) * pi_learned with alpha in [0, 1].
 */
class MpcnetBehavioralController final : public ControllerBase {
 public:
  /**
   * Constructor.
   * @param [in] alpha : The mixture parameter.
   * @param [in] optimalController : The optimal controller (this class takes ownership of a clone).
   * @param [in] learnedController : The learned controller (this class takes ownership of a clone).
   */
  MpcnetBehavioralController(scalar_t alpha, const ControllerBase& optimalController, const MpcnetControllerBase& learnedController)
      : alpha_(alpha), optimalControllerPtr_(optimalController.clone()), learnedControllerPtr_(learnedController.clone()) {}

  MpcnetBehavioralController() = default;
  ~MpcnetBehavioralController() override = default;
  MpcnetBehavioralController* clone() const override { return new MpcnetBehavioralController(*this); }

  /**
   * Set the mixture parameter.
   * @param [in] alpha : The mixture parameter.
   */
  void setAlpha(scalar_t alpha) { alpha_ = alpha; }

  /**
   * Set the optimal controller.
   * @param [in] optimalController : The optimal controller (this class takes ownership of a clone).
   */
  void setOptimalController(const ControllerBase& optimalController) { optimalControllerPtr_.reset(optimalController.clone()); }

  /**
   * Set the learned controller.
   * @param [in] learnedController : The learned controller (this class takes ownership of a clone).
   */
  void setLearnedController(const MpcnetControllerBase& learnedController) { learnedControllerPtr_.reset(learnedController.clone()); }

  vector_t computeInput(scalar_t t, const vector_t& x) override;
  ControllerType getType() const override { return ControllerType::BEHAVIORAL; }

  int size() const override;
  void clear() override;
  bool empty() const override;
  void concatenate(const ControllerBase* otherController, int index, int length) override;

 private:
  MpcnetBehavioralController(const MpcnetBehavioralController& other)
      : MpcnetBehavioralController(other.alpha_, *other.optimalControllerPtr_, *other.learnedControllerPtr_) {}

  scalar_t alpha_;
  std::unique_ptr<ControllerBase> optimalControllerPtr_;
  std::unique_ptr<MpcnetControllerBase> learnedControllerPtr_;
};

}  // namespace mpcnet
}  // namespace ocs2
