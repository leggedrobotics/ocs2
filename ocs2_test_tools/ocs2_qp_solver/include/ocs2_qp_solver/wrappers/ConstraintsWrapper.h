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

#include <ocs2_core/constraint/ConstraintBase.h>

#include <ocs2_qp_solver/QpSolverTypes.h>

namespace ocs2 {
namespace qp_solver {

/**
 * Wrapper class that wraps a ConstraintBase of any size and provides a dynamic size interface.
 * The wrapper clones the constraint function upon construction, and owns the clone.
 * This class is not thread safe, because the underlying constraint function is not thread safe.
 */
class ConstraintsWrapper {
 public:
  /** templated constructor to accept constraint function of any size */
  template <size_t STATE_DIM, size_t INPUT_DIM>
  ConstraintsWrapper(const ConstraintBase<STATE_DIM, INPUT_DIM>& constraints)  // NOLINT(google-explicit-constructor)
      : p_(new ConstraintsHandle<STATE_DIM, INPUT_DIM>(constraints)) {}

  /** Copy constructor clones the underlying handle and constraint */
  ConstraintsWrapper(const ConstraintsWrapper& other) : p_(other.p_->clone()) {}

  /** Copy constructor clones the underlying handle and constraint */
  ConstraintsWrapper& operator=(const ConstraintsWrapper& other) {
    *this = ConstraintsWrapper(other);
    return *this;
  }

  /** Move constructor moves the constraint */
  ConstraintsWrapper(ConstraintsWrapper&&) noexcept = default;

  /** Move assignments moves the constraint */
  ConstraintsWrapper& operator=(ConstraintsWrapper&&) noexcept = default;

  /** Evaluate the initial constraint */
  dynamic_vector_t getInitialConstraint(scalar_t t, const dynamic_vector_t& x, const dynamic_vector_t& u);

  /** Gets the initial constraint approximation */
  VectorFunctionLinearApproximation getInitialLinearApproximation(scalar_t t, const dynamic_vector_t& x, const dynamic_vector_t& u);

  /** Evaluate the constraint */
  dynamic_vector_t getConstraint(scalar_t t, const dynamic_vector_t& x, const dynamic_vector_t& u);

  /** Gets the constraint approximation */
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t t, const dynamic_vector_t& x, const dynamic_vector_t& u);

  /** Evaluate the terminal constraint */
  dynamic_vector_t getTerminalConstraint(scalar_t t, const dynamic_vector_t& x);

  /** Gets the terminal constraint approximation */
  VectorFunctionLinearApproximation getTerminalLinearApproximation(scalar_t t, const dynamic_vector_t& x);

 private:
  /** Base class for a handle, virtualizes the access to the templated constraint function */
  struct ConstraintsHandleBase {
    virtual ~ConstraintsHandleBase() = default;
    virtual std::unique_ptr<ConstraintsHandleBase> clone() const = 0;
    virtual void setInitialStateAndControl(scalar_t t, const dynamic_vector_t& x, const dynamic_vector_t& u) = 0;
    virtual void setCurrentStateAndControl(scalar_t t, const dynamic_vector_t& x, const dynamic_vector_t& u) = 0;
    virtual void setTerminalState(scalar_t t, const dynamic_vector_t& x) = 0;
    virtual dynamic_vector_t getConstraints() = 0;  // constraint = [StateInputConstraint; StateOnlyConstraint]
    virtual dynamic_matrix_t getConstraintsDerivativeState() = 0;
    virtual dynamic_matrix_t getConstraintsDerivativeInput() = 0;
    virtual dynamic_vector_t getTerminalConstraints() = 0;
    virtual dynamic_matrix_t getTerminalConstraintsDerivativeState() = 0;
  };
  /** Only data member: contains a polymorphic handle that wraps the constraint function */
  std::unique_ptr<ConstraintsHandleBase> p_;

  /** templated handle, containing the pointer to the actually passed constraint function */
  template <size_t STATE_DIM, size_t INPUT_DIM>
  struct ConstraintsHandle : public ConstraintsHandleBase {
    // declare const size types
    using ConstraintsFunction_t = ConstraintBase<STATE_DIM, INPUT_DIM>;
    using scalar_t = typename ConstraintsFunction_t::scalar_t;
    using state_vector_t = typename ConstraintsFunction_t::state_vector_t;
    using state_matrix_t = typename ConstraintsFunction_t::state_matrix_t;
    using input_vector_t = typename ConstraintsFunction_t::input_vector_t;
    using input_matrix_t = typename ConstraintsFunction_t::input_matrix_t;
    using input_state_matrix_t = typename ConstraintsFunction_t::input_state_matrix_t;

    // Constructor of the concrete handle
    explicit ConstraintsHandle(const ConstraintBase<STATE_DIM, INPUT_DIM>& constraints) : hp_(constraints.clone()) {}
    // Clone, clones the underlying constraint function
    std::unique_ptr<ConstraintsHandleBase> clone() const override {
      return std::unique_ptr<ConstraintsHandleBase>(new ConstraintsHandle(*hp_));
    };
    // Pointer to the actual constraint function
    std::unique_ptr<ConstraintsFunction_t> hp_;
    size_t numStateInputConstraint_ = 0;
    size_t numStateOnlyConstraint_ = 0;
    size_t numStateOnlyFinalConstraint_ = 0;
    // All function below wrap fixed size functions to dynamic size
    void setInitialStateAndControl(scalar_t t, const dynamic_vector_t& x, const dynamic_vector_t& u) override {
      hp_->setCurrentStateAndControl(t, x, u);
      numStateInputConstraint_ = hp_->numStateInputConstraint(t);
      numStateOnlyConstraint_ = 0;
    }
    void setCurrentStateAndControl(scalar_t t, const dynamic_vector_t& x, const dynamic_vector_t& u) override {
      hp_->setCurrentStateAndControl(t, x, u);
      numStateInputConstraint_ = hp_->numStateInputConstraint(t);
      numStateOnlyConstraint_ = hp_->numStateOnlyConstraint(t);
    }
    void setTerminalState(scalar_t t, const dynamic_vector_t& x) override {
      hp_->setCurrentStateAndControl(t, x, input_vector_t::Zero());
      numStateOnlyFinalConstraint_ = hp_->numStateOnlyFinalConstraint(t);
    }
    dynamic_vector_t getConstraints() override {
      // state-input equality
      typename ConstraintsFunction_t::constraint1_vector_t e;
      hp_->getConstraint1(e);
      // state-only equality
      typename ConstraintsFunction_t::constraint2_vector_t h;
      hp_->getConstraint2(h);
      // augmented constraints
      dynamic_vector_t g(numStateInputConstraint_ + numStateOnlyConstraint_);
      g << e.head(numStateInputConstraint_), h.head(numStateOnlyConstraint_);
      return g;
    }
    dynamic_matrix_t getConstraintsDerivativeState() override {
      // state-input equality
      typename ConstraintsFunction_t::constraint1_state_matrix_t dedx;
      hp_->getConstraint1DerivativesState(dedx);
      // state-only equality
      typename ConstraintsFunction_t::constraint2_state_matrix_t dhdx;
      hp_->getConstraint2DerivativesState(dhdx);
      // augmented constraints
      dynamic_matrix_t dgdx(numStateInputConstraint_ + numStateOnlyConstraint_, STATE_DIM);
      dgdx.topRows(numStateInputConstraint_) = dedx.topRows(numStateInputConstraint_);
      dgdx.bottomRows(numStateOnlyConstraint_) = dhdx.topRows(numStateOnlyConstraint_);
      return dgdx;
    }
    dynamic_matrix_t getConstraintsDerivativeInput() override {
      // state-input equality
      typename ConstraintsFunction_t::constraint1_input_matrix_t dedu;
      hp_->getConstraint1DerivativesControl(dedu);
      // augmented constraints
      dynamic_matrix_t dgdu(numStateInputConstraint_ + numStateOnlyConstraint_, INPUT_DIM);
      dgdu.topRows(numStateInputConstraint_) = dedu.topRows(numStateInputConstraint_);
      dgdu.bottomRows(numStateOnlyConstraint_).setZero();
      return dgdu;
    }
    dynamic_vector_t getTerminalConstraints() override {
      // state-only equality
      typename ConstraintsFunction_t::constraint2_vector_t h;
      hp_->getFinalConstraint2(h);
      return h.head(numStateOnlyFinalConstraint_);
    }
    dynamic_matrix_t getTerminalConstraintsDerivativeState() override {
      // state-only equality
      typename ConstraintsFunction_t::constraint2_state_matrix_t dhdx;
      hp_->getFinalConstraint2DerivativesState(dhdx);
      return dhdx.topRows(numStateOnlyFinalConstraint_);
    }
  };
};

}  // namespace qp_solver
}  // namespace ocs2
