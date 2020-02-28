//
// Created by rgrandia on 25.02.20.
//

#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBase.h>

#include <ocs2_qp_solver/QpSolverTypes.h>

namespace ocs2_qp_solver {

/**
 * Wrapper class that wraps a SystemDynamicsBase of any size and provides a dynamic size interface.
 * The wrapper clones the system dynamics upon construction, and owns the clone.
 * This class is not thread safe, because the underlying system is not thread safe.
 */
class SystemWrapper {
 public:
  /** templated constructor to accept cost function of any size */
  template <size_t STATE_DIM, size_t INPUT_DIM>
  SystemWrapper(const ocs2::SystemDynamicsBase<STATE_DIM, INPUT_DIM>& systemDynamics)  // NOLINT(google-explicit-constructor)
      : p_(new SystemHandle<STATE_DIM, INPUT_DIM>(systemDynamics)) {}

  /** Copy operations clone the underlying handle and system dynamics */
  SystemWrapper(const SystemWrapper& other) : p_(other.p_->clone()) {}
  SystemWrapper& operator=(const SystemWrapper& other) {
    *this = SystemWrapper(other);
    return *this;
  }

  /** Move operations move the handle */
  SystemWrapper(SystemWrapper&&) noexcept = default;
  SystemWrapper& operator=(SystemWrapper&&) noexcept = default;

  /** system dynamics interface */
  Eigen::VectorXd getFlowMap(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u);
  LinearDynamics getLinearApproximation(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u);

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
  /** Only data member: contains a polymorphic handle that wraps the cost function */
  std::unique_ptr<SystemHandleBase> p_;

  /** templated handle, containing the pointer to the actually passed cost function */
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
      state_matrix_t A;
      hp_->getFlowMapDerivativeState(A);
      return A;
    }
    Eigen::MatrixXd flowMapDerivativeInput() override {
      state_input_matrix_t B;
      hp_->getFlowMapDerivativeInput(B);
      return B;
    }
  };
};

}  // namespace ocs2_qp_solver
