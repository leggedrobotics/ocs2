#pragma once

#include <ocs2_switched_model_interface/core/ComModelBase.h>
#include <ocs2_switched_model_interface/core/KinematicsModelBase.h>

namespace switched_model {

template <typename Constraint>
int evaluateConstraint(ComModelBase<ocs2::CppAdInterface::ad_scalar_t>& comModel,
                       KinematicsModelBase<ocs2::CppAdInterface::ad_scalar_t>& kinModel, const typename Constraint::settings_t& settings) {
  // Set up settings
  using ad_com_model_t = ComModelBase<ad_scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ad_scalar_t>;

  // Initialize the Constraint
  Constraint constraint(0, settings, comModel, kinModel, true);

  // evaluation point
  double t = 0.0;
  input_vector_t u;
  state_vector_t x;
  u.setRandom();
  x.setRandom();

  auto order = constraint.getOrder();
  switch (order) {
    case ocs2::ConstraintOrder::Quadratic: {
      std::cerr << "\nConstraintOrder: quadratic.\n";
      auto quadApprox = constraint.getQuadraticApproximation(t, x, u);
      std::cerr << "ddhdxdx" << std::endl;
      int count = 0;
      for (auto ddhdxdx : quadApprox.dfdxx) {
        std::cerr << "\t ddhdxdx[" << count << "]" << std::endl;
        std::cerr << ddhdxdx << std::endl;
      }

      std::cerr << "ddhdudu" << std::endl;
      count = 0;
      for (auto ddhdudu : quadApprox.dfduu) {
        std::cerr << "\t ddhdudu[" << count << "]" << std::endl;
        std::cerr << ddhdudu << std::endl;
      }

      std::cerr << "ddhdudx" << std::endl;
      count = 0;
      for (auto ddhdudx : quadApprox.dfdux) {
        std::cerr << "\t ddhdudx[" << count << "]" << std::endl;
        std::cerr << ddhdudx << std::endl;
      }

      std::cerr << "dhdx: \n" << quadApprox.dfdx << std::endl;
      std::cerr << "dhdu: \n" << quadApprox.dfdu << std::endl;
      std::cerr << "h: " << quadApprox.f.transpose() << std::endl;
    }  // fallthrough on purpose
    case ocs2::ConstraintOrder::Linear: {
      std::cerr << "\nConstraintOrder: Linear.\n";
      auto linApprox = constraint.getLinearApproximation(t, x, u);
      std::cerr << "dhdx: \n" << linApprox.dfdx << std::endl;
      std::cerr << "dhdu: \n" << linApprox.dfdu << std::endl;
      std::cerr << "h: " << linApprox.f.transpose() << std::endl;
      break;
    }
    default:
      throw std::runtime_error("None or No ConstraintOrder");
      break;
  }
  return true;
}

}  // namespace switched_model
