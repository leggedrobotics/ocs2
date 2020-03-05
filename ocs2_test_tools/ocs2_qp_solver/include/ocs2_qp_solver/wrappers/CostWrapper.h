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
  double getCost(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u);

  /** Gets the cost approximation */
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u);

  /** Evaluate the terminal cost */
  double getTerminalCost(double t, const Eigen::VectorXd& x);

  /** Gets the terminal cost approximation */
  ScalarFunctionQuadraticApproximation getTerminalQuadraticApproximation(double t, const Eigen::VectorXd& x);

 private:
  /** Base class for a handle, virtualizes the access to the templated cost function */
  struct CostHandleBase {
    virtual ~CostHandleBase() = default;
    virtual std::unique_ptr<CostHandleBase> clone() const = 0;
    virtual void setCurrentStateAndControl(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) = 0;
    virtual void setCurrentStateAndControl(double t, const Eigen::VectorXd& x) = 0;
    virtual double getCost() = 0;
    virtual Eigen::VectorXd getCostDerivativeState() = 0;
    virtual Eigen::VectorXd getCostDerivativeInput() = 0;
    virtual Eigen::MatrixXd getCostSecondDerivativeState() = 0;
    virtual Eigen::MatrixXd getCostSecondDerivativeInput() = 0;
    virtual Eigen::MatrixXd getCostDerivativeInputState() = 0;
    virtual double getTerminalCost() = 0;
    virtual Eigen::VectorXd getTerminalCostDerivativeState() = 0;
    virtual Eigen::MatrixXd getTerminalCostSecondDerivativeState() = 0;
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
    void setCurrentStateAndControl(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) override {
      hp_->setCurrentStateAndControl(t, x, u);
    }
    void setCurrentStateAndControl(double t, const Eigen::VectorXd& x) override {
      hp_->setCurrentStateAndControl(t, x, input_vector_t::Zero());
    }
    double getCost() override {
      scalar_t f;
      hp_->getIntermediateCost(f);
      return f;
    }
    Eigen::VectorXd getCostDerivativeState() override {
      state_vector_t dfdx;
      hp_->getIntermediateCostDerivativeState(dfdx);
      return dfdx;
    }
    Eigen::VectorXd getCostDerivativeInput() override {
      input_vector_t dfdu;
      hp_->getIntermediateCostDerivativeInput(dfdu);
      return dfdu;
    }
    Eigen::MatrixXd getCostSecondDerivativeState() override {
      state_matrix_t dfdxx;
      hp_->getIntermediateCostSecondDerivativeState(dfdxx);
      return dfdxx;
    }
    Eigen::MatrixXd getCostSecondDerivativeInput() override {
      input_matrix_t dfduu;
      hp_->getIntermediateCostSecondDerivativeInput(dfduu);
      return dfduu;
    }
    Eigen::MatrixXd getCostDerivativeInputState() override {
      input_state_matrix_t dfdux;
      hp_->getIntermediateCostDerivativeInputState(dfdux);
      return dfdux;
    };
    double getTerminalCost() override {
      scalar_t f;
      hp_->getTerminalCost(f);
      return f;
    };
    Eigen::VectorXd getTerminalCostDerivativeState() override {
      state_vector_t dfdx;
      hp_->getTerminalCostDerivativeState(dfdx);
      return dfdx;
    };
    Eigen::MatrixXd getTerminalCostSecondDerivativeState() override {
      state_matrix_t dfdxx;
      hp_->getTerminalCostSecondDerivativeState(dfdxx);
      return dfdxx;
    };
  };
};

}  // namespace qp_solver
}  // namespace ocs2
