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

#pragma once

#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/loopshaping/LoopshapingDefinition.h>

namespace ocs2 {

class LoopshapingConstraint : public ConstraintBase {
 public:
  ~LoopshapingConstraint() override = default;

  static std::unique_ptr<LoopshapingConstraint> create(const ConstraintBase& systemConstraint,
                                                       std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

  static std::unique_ptr<LoopshapingConstraint> create(std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

  vector_t stateEqualityConstraint(scalar_t t, const vector_t& x) final;
  vector_t inequalityConstraint(scalar_t t, const vector_t& x, const vector_t& u) final;
  vector_t finalStateEqualityConstraint(scalar_t t, const vector_t& x) final;

  VectorFunctionLinearApproximation stateEqualityConstraintLinearApproximation(scalar_t t, const vector_t& x) final;
  VectorFunctionLinearApproximation finalStateEqualityConstraintLinearApproximation(scalar_t t, const vector_t& x) final;

 protected:
  LoopshapingConstraint(const ConstraintBase& systemConstraint, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : systemConstraint_(systemConstraint.clone()), loopshapingDefinition_(std::move(loopshapingDefinition)){};

  explicit LoopshapingConstraint(std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : systemConstraint_(new ConstraintBase()), loopshapingDefinition_(std::move(loopshapingDefinition)){};

  LoopshapingConstraint(const LoopshapingConstraint& obj) : ConstraintBase(obj), loopshapingDefinition_(obj.loopshapingDefinition_) {
    systemConstraint_.reset(obj.systemConstraint_->clone());
  }

 protected:
  std::unique_ptr<ConstraintBase> systemConstraint_;
  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
};

}  // namespace ocs2
