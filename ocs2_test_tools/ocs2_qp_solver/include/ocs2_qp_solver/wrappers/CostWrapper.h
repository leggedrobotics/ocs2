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

//
// Created by rgrandia on 25.02.20.
//

#pragma once

#include <ocs2_core/cost/CostFunctionBase.h>

#include <ocs2_qp_solver/QpSolverTypes.h>

namespace ocs2 {
namespace qp_solver {

/**
 * Wrapper class that wraps a CostFunctionBase of any size and provides a dynamic size interface.
 * The wrapper clones the cost function upon construction, and owns the clone.
 * This class is not thread safe, because the underlying cost function is not thread safe.
 */
class CostWrapper {
 public:
  /** templated constructor to accept cost function of any size */
  template <size_t STATE_DIM, size_t INPUT_DIM>
  CostWrapper(const ocs2::CostFunctionBase<STATE_DIM, INPUT_DIM>& costFunction)  // NOLINT(google-explicit-constructor)
      : p_(new CostHandle<STATE_DIM, INPUT_DIM>(costFunction)) {}

  /** Copy constructor clones the underlying handle and cost */
  CostWrapper(const CostWrapper& other) : p_(other.p_->clone()) {}

  /** Copy constructor clones the underlying handle and cost */
  CostWrapper& operator=(const CostWrapper& other) {
    *this = CostWrapper(other);
    return *this;
  }

  /** Move constructor moves the cost */
  CostWrapper(CostWrapper&&) noexcept = default;

  /** Move assignments moves the cost */
  CostWrapper& operator=(CostWrapper&&) noexcept = default;

  /** Evaluate the cost */
  scalar_t getCost(scalar_t t, const dynamic_vector_t& x, const dynamic_vector_t& u);

  /** Gets the cost approximation */
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t t, const dynamic_vector_t& x, const dynamic_vector_t& u);

  /** Evaluate the terminal cost */
  scalar_t getTerminalCost(scalar_t t, const dynamic_vector_t& x);

  /** Gets the terminal cost approximation */
  ScalarFunctionQuadraticApproximation getTerminalQuadraticApproximation(scalar_t t, const dynamic_vector_t& x);

 private:
  /** Base class for a handle, virtualizes the access to the templated cost function */
  struct CostHandleBase {
    virtual ~CostHandleBase() = default;
    virtual std::unique_ptr<CostHandleBase> clone() const = 0;
    virtual void setCurrentStateAndControl(scalar_t t, const dynamic_vector_t& x, const dynamic_vector_t& u) = 0;
    virtual void setCurrentStateAndControl(scalar_t t, const dynamic_vector_t& x) = 0;
    virtual scalar_t getCost() = 0;
    virtual dynamic_vector_t getCostDerivativeState() = 0;
    virtual dynamic_vector_t getCostDerivativeInput() = 0;
    virtual dynamic_matrix_t getCostSecondDerivativeState() = 0;
    virtual dynamic_matrix_t getCostSecondDerivativeInput() = 0;
    virtual dynamic_matrix_t getCostDerivativeInputState() = 0;
    virtual scalar_t getTerminalCost() = 0;
    virtual dynamic_vector_t getTerminalCostDerivativeState() = 0;
    virtual dynamic_matrix_t getTerminalCostSecondDerivativeState() = 0;
  };
  /** Only data member: contains a polymorphic handle that wraps the cost function */
  std::unique_ptr<CostHandleBase> p_;

  /** templated handle, containing the pointer to the actually passed cost function */
  template <size_t STATE_DIM, size_t INPUT_DIM>
  struct CostHandle : public CostHandleBase {
    // declare const size types
    using CostFunction_t = ocs2::CostFunctionBase<STATE_DIM, INPUT_DIM>;
    using scalar_t = typename CostFunction_t::scalar_t;
    using state_vector_t = typename CostFunction_t::state_vector_t;
    using state_matrix_t = typename CostFunction_t::state_matrix_t;
    using input_vector_t = typename CostFunction_t::input_vector_t;
    using input_matrix_t = typename CostFunction_t::input_matrix_t;
    using input_state_matrix_t = typename CostFunction_t::input_state_matrix_t;

    // Constructor of the concrete handle
    explicit CostHandle(const ocs2::CostFunctionBase<STATE_DIM, INPUT_DIM>& costFunction) : hp_(costFunction.clone()) {}
    // Clone, clones the underlying cost function
    std::unique_ptr<CostHandleBase> clone() const override { return std::unique_ptr<CostHandleBase>(new CostHandle(*hp_)); };
    // Pointer to the actual cost function
    std::unique_ptr<CostFunction_t> hp_;
    // All function below wrap fixed size functions to dynamic size
    void setCurrentStateAndControl(scalar_t t, const dynamic_vector_t& x, const dynamic_vector_t& u) override {
      hp_->setCurrentStateAndControl(t, x, u);
    }
    void setCurrentStateAndControl(scalar_t t, const dynamic_vector_t& x) override {
      hp_->setCurrentStateAndControl(t, x, input_vector_t::Zero());
    }
    scalar_t getCost() override {
      scalar_t f;
      hp_->getIntermediateCost(f);
      return f;
    }
    dynamic_vector_t getCostDerivativeState() override {
      state_vector_t dfdx;
      hp_->getIntermediateCostDerivativeState(dfdx);
      return dfdx;
    }
    dynamic_vector_t getCostDerivativeInput() override {
      input_vector_t dfdu;
      hp_->getIntermediateCostDerivativeInput(dfdu);
      return dfdu;
    }
    dynamic_matrix_t getCostSecondDerivativeState() override {
      state_matrix_t dfdxx;
      hp_->getIntermediateCostSecondDerivativeState(dfdxx);
      return dfdxx;
    }
    dynamic_matrix_t getCostSecondDerivativeInput() override {
      input_matrix_t dfduu;
      hp_->getIntermediateCostSecondDerivativeInput(dfduu);
      return dfduu;
    }
    dynamic_matrix_t getCostDerivativeInputState() override {
      input_state_matrix_t dfdux;
      hp_->getIntermediateCostDerivativeInputState(dfdux);
      return dfdux;
    };
    scalar_t getTerminalCost() override {
      scalar_t f;
      hp_->getTerminalCost(f);
      return f;
    };
    dynamic_vector_t getTerminalCostDerivativeState() override {
      state_vector_t dfdx;
      hp_->getTerminalCostDerivativeState(dfdx);
      return dfdx;
    };
    dynamic_matrix_t getTerminalCostSecondDerivativeState() override {
      state_matrix_t dfdxx;
      hp_->getTerminalCostSecondDerivativeState(dfdxx);
      return dfdxx;
    };
  };
};

}  // namespace qp_solver
}  // namespace ocs2
