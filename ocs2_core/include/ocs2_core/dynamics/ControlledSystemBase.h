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

#include <memory>

#include <ocs2_core/Types.h>

#include <ocs2_core/PreComputation.h>
#include <ocs2_core/control/ControllerBase.h>
#include <ocs2_core/integration/OdeBase.h>

namespace ocs2 {

/**
 * The base class for non-autonomous system dynamics.
 */
class ControlledSystemBase : public OdeBase {
 public:
  /**
   * Constructor
   *
   * @param [in] preComputation: The (optional) pre-computation module, internally keeps a copy.
   *                             @see PreComputation class documentation.
   */
  explicit ControlledSystemBase(const PreComputation& preComputation = PreComputation());

  /** Default destructor */
  ~ControlledSystemBase() override = default;

  /** Clone */
  virtual ControlledSystemBase* clone() const = 0;

  /** Resets the internal classes. */
  virtual void reset() { controllerPtr_ = nullptr; }

  /**
   * Sets the control policy using the controller class.
   */
  void setController(ControllerBase* controllerPtr) { controllerPtr_ = controllerPtr; };

  /**
   * Returns the controller pointer.
   */
  ControllerBase* controllerPtr() const { return controllerPtr_; };

  /**
   * Computes the flow map of a system.
   *
   * @param [in] t: The current time.
   * @param [in] x: The current state.
   * @return The state time derivative.
   */
  vector_t computeFlowMap(scalar_t t, const vector_t& x) override final;

  /**
   * Computes the flow map of a system with exogenous input.
   *
   * @param [in] t: The current time.
   * @param [in] x: The current state.
   * @param [in] u: The current input.
   * @param [in] preComp: pre-computation module, safely ignore this parameter if not used.
   *                      @see PreComputation class documentation.
   * @return The state time derivative.
   */
  virtual vector_t computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComp) = 0;

  /**
   * State map at the transition time
   *
   * @param [in] time: transition time
   * @param [in] state: transition state
   * @param [in] preComp: pre-computation module, safely ignore this parameter if not used.
   *                      @see PreComputation class documentation.
   * @return mapped state after transition
   */
  virtual vector_t computeJumpMap(scalar_t time, const vector_t& state, const PreComputation& preComp);

  /**
   * Computes the flow map of a system with exogenous input.
   *
   * @note This method calls the internal preComputation request() callback and the virtual
   *       computeFlowMap() with the preComputation as parameter.
   *       This interface is used by Rollout and SensitivityIntegrator.
   */
  vector_t computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u);

  /**
   * State map at the transition time
   *
   * @note This method calls the internal preComputation requestPreJump() callback and the virtual
   *       computeJumpMap() with the preComputation as parameter.
   *       This interface is used by Rollout.
   */
  vector_t computeJumpMap(scalar_t time, const vector_t& state) override final;

  /** Get the pre-computation module */
  const PreComputation& getPreComputation() const { return *preCompPtr_; }

 protected:
  /**
   * Copy constructor
   *
   * @note Keeps the same controller pointer.
   * @note Clones the pre-computation object.
   */
  ControlledSystemBase(const ControlledSystemBase& other);

  std::unique_ptr<PreComputation> preCompPtr_;  //! pointer to pre-computation module

 private:
  ControllerBase* controllerPtr_ = nullptr;  //! pointer to controller
};

}  // namespace ocs2
