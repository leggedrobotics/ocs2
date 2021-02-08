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
#include <string>

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_core/model_data/ModelDataBase.h>

namespace ocs2 {

/**
 * This class is an interface class for constructing LQ approximation of the continous time optimal control problem.
 */
class LinearQuadraticApproximator {
 public:
  /**
   * Constructor
   *
   * @param [in] systemDerivatives: The system dynamics derivatives for subsystems of the system.
   * @param [in] systemConstraints: The system constraint function and its derivatives for subsystems.
   * @param [in] costFunction: The cost function (intermediate and terminal costs) and its derivatives for subsystems.
   * @param [in] checkNumericalCharacteristics: check for the expected numerical characteristics of the model (default true)
   */
  LinearQuadraticApproximator(const SystemDynamicsBase& systemDerivatives, const ConstraintBase& systemConstraints,
                              const CostFunctionBase& costFunction, bool checkNumericalCharacteristics = true)
      : systemDynamicsPtr_(systemDerivatives.clone()),
        systemConstraintsPtr_(systemConstraints.clone()),
        costFunctionPtr_(costFunction.clone()),
        checkNumericalCharacteristics_(checkNumericalCharacteristics) {}

  /**
   * Default destructor.
   */
  ~LinearQuadraticApproximator() = default;

  /**
   * Whether or not to check the numerical characteristics of the model.
   *
   * @param [in] checkNumericalCharacteristics: True if the numerical characteristics of the model should be checked.
   */
  void checkNumericalCharacteristics(bool checkNumericalCharacteristics) { checkNumericalCharacteristics_ = checkNumericalCharacteristics; }

  /**
   * Returns the system derivatives
   */
  SystemDynamicsBase& systemDerivatives() const { return *systemDynamicsPtr_; }

  /**
   * Returns the constraints.
   */
  ConstraintBase& systemConstraints() const { return *systemConstraintsPtr_; }

  /**
   * Returns the intermediate cost.
   */
  CostFunctionBase& costFunction() const { return *costFunctionPtr_; }

  /**
   * Calculates an LQ approximate of the constrained optimal control problem at a given time, state, and input.
   *
   * @param [in] time: The current time.
   * @param [in] state: The current state.
   * @param [in] input: The current input.
   * @param [out] modelData: The output data model.
   */
  void approximateLQProblem(const scalar_t& time, const vector_t& state, const vector_t& input, ModelDataBase& modelData) const;

  /**
   * Calculates an LQ approximate of the constrained optimal control problem at an event time.
   *
   * @param [in] time: The current time.
   * @param [in] state: The current state.
   * @param [in] input: The current input.
   * @param [out] modelData: The output data model.
   */
  void approximateLQProblemAtEventTime(const scalar_t& time, const vector_t& state, const vector_t& input, ModelDataBase& modelData) const;

  /**
   * Calculates linearized system dynamics.
   *
   * @param [in] time: The current time.
   * @param [in] state: The current state.
   * @param [in] input: The current input.
   * @param [in] modelData: Model data object.
   */
  void approximateDynamics(const scalar_t& time, const vector_t& state, const vector_t& input, ModelDataBase& modelData) const;

  /**
   * Calculates the constraints and its linearized approximation.
   *
   * @param [in] time: The current time.
   * @param [in] state: The current state.
   * @param [in] input: The current input.
   * @param [in] modelData: Model data object.
   */
  void approximateConstraints(const scalar_t& time, const vector_t& state, const vector_t& input, ModelDataBase& modelData) const;

  /**
   * Calculates the cost function and its quadratic approximation.
   *
   * @param [in] time: The current time.
   * @param [in] state: The current state.
   * @param [in] input: The current input.
   * @param [in] modelData: Model data object.
   */
  void approximateCost(const scalar_t& time, const vector_t& state, const vector_t& input, ModelDataBase& modelData) const;

 private:
  std::unique_ptr<SystemDynamicsBase> systemDynamicsPtr_;
  std::unique_ptr<ConstraintBase> systemConstraintsPtr_;
  std::unique_ptr<CostFunctionBase> costFunctionPtr_;

  bool checkNumericalCharacteristics_;
};

}  // namespace ocs2
