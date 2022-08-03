/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/PreComputation.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_core/model_data/Metrics.h>
#include <ocs2_core/model_data/Multiplier.h>

namespace ocs2 {

/** The base class for Augmented Lagrangian penalty of state-input constraint. */
class StateInputAugmentedLagrangianInterface {
 public:
  StateInputAugmentedLagrangianInterface() = default;
  virtual ~StateInputAugmentedLagrangianInterface() = default;
  virtual StateInputAugmentedLagrangianInterface* clone() const = 0;

  /** Check penalty's activity */
  virtual bool isActive(scalar_t time) const = 0;

  /** Get the size of the constraint vector at given time */
  virtual size_t getNumConstraints(scalar_t time) const = 0;

  /** Get the constraint and its penalty value */
  virtual LagrangianMetrics getValue(scalar_t time, const vector_t& state, const vector_t& input, const Multiplier& lagrangian,
                                     const PreComputation& preComp) const = 0;

  /** Get the constraint's penalty quadratic approximation */
  virtual ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                         const Multiplier& lagrangian,
                                                                         const PreComputation& preComp) const = 0;

  /** Update Lagrange/penalty multipliers and the penalty function value. */
  virtual std::pair<Multiplier, scalar_t> updateLagrangian(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const vector_t& constraint, const Multiplier& lagrangian) const = 0;

  /** Initialize Lagrange/penalty multipliers. */
  virtual Multiplier initializeLagrangian(scalar_t time) const = 0;

 protected:
  StateInputAugmentedLagrangianInterface(const StateInputAugmentedLagrangianInterface& rhs) = default;
};

}  // namespace ocs2
