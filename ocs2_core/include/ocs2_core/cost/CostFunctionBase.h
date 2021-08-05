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

#include <ocs2_core/reference/TargetTrajectories.h>
#include "ocs2_core/Types.h"

namespace ocs2 {

/**
 * Cost Function base class.
 */
class CostFunctionBase {
 public:
  /** Constructor */
  CostFunctionBase() = default;

  /** Copy constructor */
  CostFunctionBase(const CostFunctionBase& rhs) = default;

  /** Default destructor */
  virtual ~CostFunctionBase() = default;

  /**
   * Sets the TargetTrajectories in the cost function.
   * @param [in] targetTrajectoriesPtr: A cost pointer to TargetTrajectories.
   */
  virtual void setTargetTrajectoriesPtr(const TargetTrajectories* targetTrajectoriesPtr) { targetTrajectoriesPtr_ = targetTrajectoriesPtr; }

  /** Clone */
  virtual CostFunctionBase* clone() const = 0;

  /** Evaluate the cost */
  virtual scalar_t cost(scalar_t t, const vector_t& x, const vector_t& u) = 0;

  /** Evaluate the final cost */
  virtual scalar_t finalCost(scalar_t t, const vector_t& x) = 0;

  /** Evaluate the cost quadratic approximation */
  virtual ScalarFunctionQuadraticApproximation costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) = 0;

  /** Evaluate the final cost quadratic approximation */
  virtual ScalarFunctionQuadraticApproximation finalCostQuadraticApproximation(scalar_t t, const vector_t& x) = 0;

  /** Time derivative of the intermediate cost */
  virtual scalar_t costDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) { return 0; }

  /** Time derivative of final cost */
  virtual scalar_t finalCostDerivativeTime(scalar_t t, const vector_t& x) { return 0; }

 protected:
  const TargetTrajectories* targetTrajectoriesPtr_ = nullptr;
};

}  // namespace ocs2
