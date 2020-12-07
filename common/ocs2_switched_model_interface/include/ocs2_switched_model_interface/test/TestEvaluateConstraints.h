#pragma once

#include <ocs2_switched_model_interface/constraint/ConstraintTerm.h>
#include <ocs2_switched_model_interface/core/ComModelBase.h>
#include <ocs2_switched_model_interface/core/KinematicsModelBase.h>

namespace switched_model {

template <typename Constraint>
int evaluateConstraint(ComModelBase<ocs2::CppAdInterface::ad_scalar_t>& comModel,
                       KinematicsModelBase<ocs2::CppAdInterface::ad_scalar_t>& kinModel) {
  // Set up settings
  using ad_base_t = CppAD::cg::CG<scalar_t>;
  using ad_scalar_t = CppAD::AD<ad_base_t>;
  using ad_com_model_t = ComModelBase<ad_scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ad_scalar_t>;

  typename Constraint::settings_t settings(3, 3);
  settings.A.setIdentity(3, 3);
  settings.b.setZero();

  // Initialize the Constraint
  Constraint constraint(0, settings, comModel, kinModel, true);

  // evaluation point
  double t = 0.0;
  typename Constraint::input_vector_t u;
  typename Constraint::state_vector_t x;
  u.setRandom();
  x.setRandom();

  auto order = constraint.getOrder();
  switch (order) {
    case ConstraintOrder::Quadratic: {
      std::cout << "\nConstraintOrder: quadratic.\n";
      auto quadApprox = (constraint.getQuadraticApproximation(t, x, u));
      std::cout << "ddhdxdx" << std::endl;
      int count = 0;
      for (auto ddhdxdx : quadApprox.secondDerivativesState) {
        std::cout << "\t ddhdxdx[" << count << "]" << std::endl;
        std::cout << ddhdxdx << std::endl;
      }

      std::cout << "ddhdudu" << std::endl;
      count = 0;
      for (auto ddhdudu : quadApprox.secondDerivativesInput) {
        std::cout << "\t ddhdudu[" << count << "]" << std::endl;
        std::cout << ddhdudu << std::endl;
      }

      std::cout << "ddhdudx" << std::endl;
      count = 0;
      for (auto ddhdudx : quadApprox.derivativesInputState) {
        std::cout << "\t ddhdudx[" << count << "]" << std::endl;
        std::cout << ddhdudx << std::endl;
      }
      for (auto dhdx : quadApprox.derivativeState) {
        std::cout << dhdx.transpose() << std::endl;
      }

      std::cout << "dhdu" << std::endl;
      for (auto dhdu : quadApprox.derivativeInput) {
        std::cout << dhdu.transpose() << std::endl;
      }

      std::cout << "h" << std::endl;
      for (auto h : quadApprox.constraintValues) {
        std::cout << h << std::endl;
      }
    }
      //! fallthrough on purpose
    case ConstraintOrder::Linear: {
      std::cout << "\nConstraintOrder: Linear.\n";
      auto linApprox = (constraint.getLinearApproximation(t, x, u));
      std::cout << "dhdx" << std::endl;
      for (auto dhdx : linApprox.derivativeState) {
        std::cout << dhdx.transpose() << std::endl;
      }

      std::cout << "dhdu" << std::endl;
      for (auto dhdu : linApprox.derivativeInput) {
        std::cout << dhdu.transpose() << std::endl;
      }

      std::cout << "h" << std::endl;
      for (auto h : linApprox.constraintValues) {
        std::cout << h << std::endl;
      }
      break;
    }
    default:
      throw std::runtime_error("None or No ConstraintOrder");
      break;
  }
  return true;
}

}  // namespace switched_model
