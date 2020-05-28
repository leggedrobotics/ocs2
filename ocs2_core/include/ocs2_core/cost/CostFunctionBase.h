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

#include "ocs2_core/Types.h"
#include "ocs2_core/cost/CostDesiredTrajectories.h"

namespace ocs2 {

/**
 * Cost Function Base.
 */
class CostFunctionBase {
 public:
  /**
   * Constructor
   */
  CostFunctionBase();

  /**
   * Copy constructor
   */
  CostFunctionBase(const CostFunctionBase& rhs);

  /**
   * Default destructor
   */
  virtual ~CostFunctionBase() = default;

  /**
   * Sets the desired state and input trajectories used in the cost function.
   *
   * @param [in] CostDesiredTrajectoriesPtr: A cost pointer to desired trajectories.
   */
  virtual void setCostDesiredTrajectoriesPtr(const CostDesiredTrajectories* costDesiredTrajectoriesPtr);

  /**
   * Returns pointer to the class.
   *
   * @return A raw pointer to the class.
   */
  virtual CostFunctionBase* clone() const = 0;

  /**
   * Sets the current time, state, and control input
   *
   * @param [in] t: Current time
   * @param [in] x: Current state vector
   * @param [in] u: Current input vector
   */
  virtual void setCurrentStateAndControl(scalar_t t, const vector_t& x, const vector_t& u);

  /**
   * Get the intermediate cost.
   *
   * @param [out] L: The intermediate cost value.
   */
  virtual scalar_t getIntermediateCost() = 0;

  /**
   * Get the time derivative of the intermediate cost.
   *
   * @return The time derivative of intermediate cost.
   */
  virtual scalar_t getIntermediateCostDerivativeTime();

  /**
   * Get the state derivative of the intermediate cost.
   *
   * @return First order derivative of the intermediate cost with respect to state vector, size \f$ n_x \f$.
   */
  virtual vector_t getIntermediateCostDerivativeState() = 0;

  /**
   * Get state second order derivative of the intermediate cost.
   *
   * @return Second order derivative of the intermediate cost with respect to state vector, size \f$ n_x * n_x \f$.
   */
  virtual matrix_t getIntermediateCostSecondDerivativeState() = 0;

  /**
   * Get control input derivative of the intermediate cost.
   *
   * @return First order derivative of the intermediate cost with respect to input vector, size \f$ n_u \f$.
   */
  virtual vector_t getIntermediateCostDerivativeInput() = 0;

  /**
   * Get control input second derivative of the intermediate cost.
   *
   * @return Second order derivative of the intermediate cost with respect to input vector, size \f$ n_u * n_u \f$.
   */
  virtual matrix_t getIntermediateCostSecondDerivativeInput() = 0;

  /**
   * Get the input-state derivative of the intermediate cost.
   *
   * @return Second order derivative of the intermediate cost with respect to input vector and state, size \f$ n_u * n_x \f$.
   */
  virtual matrix_t getIntermediateCostDerivativeInputState() = 0;

  /**
   * Get the terminal cost.
   *
   * @return The final cost value.
   */
  virtual scalar_t getTerminalCost() = 0;

  /**
   * Get the time derivative of terminal cost.
   *
   * @return The time derivative of terminal cost.
   */
  virtual scalar_t getTerminalCostDerivativeTime();

  /**
   * Get the terminal cost state derivative of the terminal cost.
   *
   * @return First order final cost derivative with respect to state vector, size \f$ n_x \f$.
   */
  virtual vector_t getTerminalCostDerivativeState() = 0;

  /**
   * Get the terminal cost state second derivative of the terminal cost.
   *
   * @return Second order final cost derivative with respect to state vector, size \f$ n_x * n_x \f$.
   */
  virtual matrix_t getTerminalCostSecondDerivativeState() = 0;

 protected:
  const CostDesiredTrajectories* costDesiredTrajectoriesPtr_;

  scalar_t t_;
  vector_t x_;
  vector_t u_;
};

}  // namespace ocs2
