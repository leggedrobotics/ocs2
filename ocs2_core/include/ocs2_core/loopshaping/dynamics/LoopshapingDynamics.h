/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_core/loopshaping/LoopshapingDefinition.h>
#include <ocs2_core/loopshaping/LoopshapingPreComputation.h>

namespace ocs2 {
class LoopshapingDynamics : public SystemDynamicsBase {
 public:
  ~LoopshapingDynamics() override = default;

  static std::unique_ptr<LoopshapingDynamics> create(const SystemDynamicsBase& systemDynamics,
                                                     std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

  vector_t computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) final;
  vector_t computeJumpMap(scalar_t time, const vector_t& state, const PreComputation& preComp) final;
  vector_t computeGuardSurfaces(scalar_t time, const vector_t& state) final;

  VectorFunctionLinearApproximation jumpMapLinearApproximation(scalar_t t, const vector_t& x, const PreComputation& preComp) final;
  VectorFunctionLinearApproximation guardSurfacesLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u) final;

  vector_t flowMapDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) final;
  vector_t jumpMapDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) final;
  vector_t guardSurfacesDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) final;

 protected:
  LoopshapingDynamics(const LoopshapingDynamics& other)
      : SystemDynamicsBase(other), systemDynamics_(other.systemDynamics_->clone()), loopshapingDefinition_(other.loopshapingDefinition_) {}

  LoopshapingDynamics(const SystemDynamicsBase& systemDynamics, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition,
                      const LoopshapingPreComputation& preComputation)
      : SystemDynamicsBase(preComputation),
        systemDynamics_(systemDynamics.clone()),
        loopshapingDefinition_(std::move(loopshapingDefinition)) {}

  std::unique_ptr<SystemDynamicsBase> systemDynamics_;
  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;

 private:
  virtual vector_t filterFlowmap(const vector_t& x_filter, const vector_t& u_filter, const vector_t& u_system) = 0;
};

}  // namespace ocs2
