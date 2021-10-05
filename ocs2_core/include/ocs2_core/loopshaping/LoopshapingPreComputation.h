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

#include <ocs2_core/PreComputation.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/loopshaping/LoopshapingDefinition.h>

namespace ocs2 {

/**
 * Loopshaping Pre-Computation decorator class.
 */
class LoopshapingPreComputation final : public PreComputation {
 public:
  LoopshapingPreComputation(const PreComputation& systemPreComputation, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);
  ~LoopshapingPreComputation() override = default;
  LoopshapingPreComputation* clone() const override;

  void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;
  void requestPreJump(RequestSet request, scalar_t t, const vector_t& x) override;
  void requestFinal(RequestSet request, scalar_t t, const vector_t& x) override;

  /** System state, computed for the last request. */
  const vector_t& getSystemState() const { return systemState_; }

  /** System input, computed for the last request. */
  const vector_t& getSystemInput() const { return systemInput_; }

  /** Filter state, computed for the last request. */
  const vector_t& getFilterState() const { return filterState_; }

  /** Filter Input, computed for the last request. */
  const vector_t& getFilteredInput() const { return filterInput_; }

  /** Precomputation evaluated for the system state and system input */
  const PreComputation& getSystemPreComputation() const { return *systemPreCompPtr_; }

 private:
  LoopshapingPreComputation(const LoopshapingPreComputation& rhs);

  vector_t systemState_;
  vector_t systemInput_;
  vector_t filterState_;
  vector_t filterInput_;

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;

  std::unique_ptr<PreComputation> systemPreCompPtr_;
};

}  // namespace ocs2