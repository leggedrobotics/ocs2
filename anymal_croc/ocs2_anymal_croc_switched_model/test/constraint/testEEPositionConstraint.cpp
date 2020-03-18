//
// Created by rgrandia on 19.09.19.
//

#include <gtest/gtest.h>

#include <ocs2_switched_model_interface/constraint/ConstraintTerm.h>
#include <ocs2_switched_model_interface/constraint/EndEffectorPositionConstraint.h>
#include <stdexcept>

#include "ocs2_anymal_croc_switched_model/core/AnymalCrocCom.h"
#include "ocs2_anymal_croc_switched_model/core/AnymalCrocKinematics.h"

TEST(TestEEPositionConstraint, evaluate) {
  using TestedConstraint = switched_model::EndEffectorPositionConstraint;

  switched_model::EndEffectorPositionConstraintSettings settings(3, 3);
  settings.A.setIdentity(3, 3);
  settings.b.setZero();

  anymal::AnymalCrocComAd anymalComAd;
  anymal::AnymalCrocKinematicsAd anymalKinematicsAd;
  TestedConstraint eePositionConstraint(0, settings, anymalComAd, anymalKinematicsAd, true);

  // evaluation point
  double t = 0.0;
  TestedConstraint::input_vector_t u;
  TestedConstraint::state_vector_t x;
  u.setRandom();
  x.setRandom();

  switch (eePositionConstraint.getOrder()) {
    case ocs2::ConstraintOrder::Quadratic: {
      auto quadApprox = (eePositionConstraint.getQuadraticApproximation(t, x, u));
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

    case ocs2::ConstraintOrder::Linear: {
      auto linApprox = (eePositionConstraint.getLinearApproximation(t, x, u));
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
    } break;
    default:
      throw std::runtime_error("None or No ConstraintOrder");
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
