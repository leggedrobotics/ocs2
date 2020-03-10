/*!
 * @file   testEEPositionConstraint.cpp
 * @author Marko Bjelonic
 * @date   Nov 27, 2019
 */

#include <gtest/gtest.h>

#include <ocs2_switched_model_interface/constraint/EndEffectorPositionConstraint.h>
// #include <ocs2_switched_model_interface/constraint/EndEffectorInBasePositionConstraint.h>

#include "ocs2_anymal_wheels_switched_model/core/AnymalWheelsCom.h"
#include "ocs2_anymal_wheels_switched_model/core/AnymalWheelsKinematics.h"

class ConstraintsFixture : public ::testing::Test {
 public:
  template <class TestedConstraint, class Settings>
  static inline void ConstraintTest() {
    Settings settings;
    settings.A.setIdentity(3, 3);
    settings.b.setZero();

    anymal::AnymalWheelsComAd anymalComAd;
    anymal::AnymalWheelsKinematicsAd anymalKinematicsAd;
    TestedConstraint eeConstraint(0, settings, anymalComAd, anymalKinematicsAd, true);

    // evaluation point
    double t = 0.0;
    typename TestedConstraint::input_vector_t u;
    typename TestedConstraint::state_vector_t x;
    u.setRandom();
    x.setRandom();

    auto approximation = eeConstraint.getQuadraticApproximation(t, x, u);
    std::cout << "h" << std::endl;
    for (auto h : approximation.constraintValues) {
      std::cout << h << std::endl;
    }

    std::cout << "dhdx" << std::endl;
    for (auto dhdx : approximation.derivativeState) {
      std::cout << dhdx.transpose() << std::endl;
    }

    std::cout << "dhdu" << std::endl;
    for (auto dhdu : approximation.derivativeInput) {
      std::cout << dhdu.transpose() << std::endl;
    }

    std::cout << "ddhdxdx" << std::endl;
    int count = 0;
    for (auto ddhdxdx : approximation.secondDerivativesState) {
      std::cout << "\t ddhdxdx[" << count << "]" << std::endl;
      std::cout << ddhdxdx << std::endl;
    }

    std::cout << "ddhdudu" << std::endl;
    count = 0;
    for (auto ddhdudu : approximation.secondDerivativesInput) {
      std::cout << "\t ddhdudu[" << count << "]" << std::endl;
      std::cout << ddhdudu << std::endl;
    }

    std::cout << "ddhdudx" << std::endl;
    count = 0;
    for (auto ddhdudx : approximation.derivativesInputState) {
      std::cout << "\t ddhdudx[" << count << "]" << std::endl;
      std::cout << ddhdudx << std::endl;
    }
  }
};

TEST(ConstraintsFixture, TestEEPositionConstraintEvaluate) {
  using TestedConstraint = switched_model::EndEffectorPositionConstraint;
  using Settings = switched_model::EndEffectorPositionConstraintSettings;
  ConstraintsFixture::ConstraintTest<TestedConstraint, Settings>();
}

/*
 * TEST(ConstraintsFixture, TestEEPositionInBaseConstraintEvaluate) {
 *   using TestedConstraint = switched_model::EndEffectorPositionInBaseConstraint;
 *   using Settings = switched_model::EndEffectorPositionInBaseConstraintSettings;
 *   ConstraintsFixture::ConstraintTest<TestedConstraint, Settings>();
 * }
 */

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
