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

#include <ocs2_core/dynamics/SystemDynamicsBase.h>

#include <ocs2_qp_solver/QpSolverTypes.h>

namespace ocs2 {
namespace qp_solver {

/**
 * Wrapper class that wraps a SystemDynamicsBase of any size and provides a dynamic size interface.
 * The wrapper clones the system dynamics upon construction, and owns the clone.
 * This class is not thread safe, because the underlying system is not thread safe.
 */
class SystemWrapper {
 public:
  /** Templated constructor to accept system dynamics of any size */
  template <size_t STATE_DIM, size_t INPUT_DIM>
  SystemWrapper(const ocs2::SystemDynamicsBase<STATE_DIM, INPUT_DIM>& systemDynamics)  // NOLINT(google-explicit-constructor)
      : p_(new SystemHandle<STATE_DIM, INPUT_DIM>(systemDynamics)) {}

  /** Copy constructor clones the underlying system dynamics */
  SystemWrapper(const SystemWrapper& other) : p_(other.p_->clone()) {}

  /** Copy assignment clones the underlying system dynamics */
  SystemWrapper& operator=(const SystemWrapper& other) {
    *this = SystemWrapper(other);
    return *this;
  }

  /** Move constructor moves the system */
  SystemWrapper(SystemWrapper&&) noexcept = default;

  /** Move assignment moves the system */
  SystemWrapper& operator=(SystemWrapper&&) noexcept = default;

  /** Evaluates the flowmap */
  Eigen::VectorXd getFlowMap(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u);

  /** Gets linear approximation */
  VectorFunctionLinearApproximation getLinearApproximation(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u);

 private:
  /** Base class for a handle, virtualizes the access to the templated system dynamics*/
  struct SystemHandleBase {
    virtual ~SystemHandleBase() = default;
    virtual std::unique_ptr<SystemHandleBase> clone() const = 0;
    virtual void setCurrentStateAndControl(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) = 0;
    virtual Eigen::VectorXd flowMap(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) = 0;
    virtual Eigen::MatrixXd flowMapDerivativeState() = 0;
    virtual Eigen::MatrixXd flowMapDerivativeInput() = 0;
  };
  /** Only data member: contains a polymorphic handle that wraps the system dynamics function */
  std::unique_ptr<SystemHandleBase> p_;

  /** Templated handle, containing the pointer to the actually passed system dynamics function */
  template <size_t STATE_DIM, size_t INPUT_DIM>
  struct SystemHandle : public SystemHandleBase {
    // declare const size types
    using SystemDynamics_t = ocs2::SystemDynamicsBase<STATE_DIM, INPUT_DIM>;
    using scalar_t = typename SystemDynamics_t::scalar_t;
    using state_vector_t = typename SystemDynamics_t::state_vector_t;
    using state_matrix_t = typename SystemDynamics_t::state_matrix_t;
    using input_vector_t = typename SystemDynamics_t::input_vector_t;
    using state_input_matrix_t = typename SystemDynamics_t::state_input_matrix_t;

    // Constructor of the concrete handle
    explicit SystemHandle(const SystemDynamics_t& systemDynamics) : hp_(systemDynamics.clone()) {}
    // Clone, clones the underlying system dynamics
    std::unique_ptr<SystemHandleBase> clone() const override { return std::unique_ptr<SystemHandleBase>(new SystemHandle(*hp_)); };
    // Pointer to the actual system dynamics
    std::unique_ptr<SystemDynamics_t> hp_;
    // All function below wrap fixed size functions to dynamic size
    void setCurrentStateAndControl(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) override {
      hp_->setCurrentStateAndControl(t, x, u);
    }
    Eigen::VectorXd flowMap(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) override {
      state_vector_t dxdt;
      hp_->computeFlowMap(t, x, u, dxdt);
      return dxdt;
    }
    Eigen::MatrixXd flowMapDerivativeState() override {
      state_matrix_t dfdx;
      hp_->getFlowMapDerivativeState(dfdx);
      return dfdx;
    }
    Eigen::MatrixXd flowMapDerivativeInput() override {
      state_input_matrix_t dfdu;
      hp_->getFlowMapDerivativeInput(dfdu);
      return dfdu;
    }
  };
};

}  // namespace qp_solver
}  // namespace ocs2
