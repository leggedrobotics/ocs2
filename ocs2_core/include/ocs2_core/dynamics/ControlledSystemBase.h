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

#include <ocs2_core/Types.h>

#include <ocs2_core/control/ControllerBase.h>
#include <ocs2_core/integration/OdeBase.h>

namespace ocs2 {

/**
 * The base class for non-autonomous system dynamics.
 */
class ControlledSystemBase : public OdeBase {
 public:
  /**
   * Constructor.
   */
  ControlledSystemBase();

  /**
   * Copy constructor.
   */
  ControlledSystemBase(const ControlledSystemBase& rhs);

  /**
   * Default destructor.
   */
  virtual ~ControlledSystemBase() = default;

  /**
   * Resets the internal classes.
   */
  virtual void reset();

  /**
   * Sets the control policy using the controller class.
   *
   * @param [in] controllerPtr: A pointer to the control policy.
   */
  void setController(ControllerBase* controllerPtr);

  /**
   * Returns the controller pointer.
   *
   * @return A pointer to controller.
   */
  ControllerBase* controllerPtr() const;

  /**
   * Computes the flow map of a system.
   *
   * @param [in] t: The current time.
   * @param [in] x: The current state.
   * @param [out] dxdt: The state time derivative.
   */
  void computeFlowMap(const scalar_t& t, const vector_t& x, vector_t& dxdt) final;

  /**
   * Computes the flow map of a system with exogenous input.
   *
   * @param [in] t: The current time.
   * @param [in] x: The current state.
   * @param [in] u: The current input.
   * @param [out] dxdt: The state time derivative.
   */
  virtual void computeFlowMap(const scalar_t& t, const vector_t& x, const vector_t& u, vector_t& dxdt) = 0;

  /**
   * Returns pointer to the class.
   *
   * @return A raw pointer to the class.
   */
  virtual ControlledSystemBase* clone() const = 0;

 private:
  ControllerBase* controllerPtr_;  //! pointer to controller
};

}  // namespace ocs2
