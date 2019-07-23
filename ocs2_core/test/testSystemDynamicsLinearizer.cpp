/*
 * testDerivativeChecker.cpp
 *
 *  Created on: 2019-06-24
 *      Author: oharley
 */

//TODO(oharley) use the correct systems (to be defined)
//TODO(oharley) experiment with the tolerances
//TODO(oharley) adhere to code conventions

#include "include/testSystemDynamicsLinearizer.h"
#include "../ocs2_core/dynamics/SystemDynamicsLinearizer.h"
#include <Eigen/Dense>
//#include "../ocs2_core/misc/DerivativeChecker.h"

#include <gtest/gtest.h>

using namespace ocs2;

class SystemDynamicsLinearizerTest : public ::testing::Test {
protected:
  void SetUp() override {
    //std::srand((unsigned int) time(0)); //Seed randomly
    std::srand((unsigned int) 0xABCDEF); //Seed repeatably
    EXP0_System linSys;
    auto linSys_ptr = std::make_shared<EXP0_System>();
    linDeriv0.reset(new EXP0_SystemDerivative(linSys_ptr));

    // Incorrect-derivative system, reuse EX0_system but with incorrect deriv
    linDeriv1.reset(new EXP1_SystemDerivative(linSys_ptr));

    // Nonlinear system
    EXP2_System nonLinSys;
    auto nonLinSys_ptr = std::make_shared<EXP2_System>();
    nonLinDeriv.reset(new EXP2_SystemDerivative(nonLinSys_ptr));

    // Nonlinear system, unstable
    nonLinDerivUnstable.reset(new EXP3_SystemDerivative(nonLinSys_ptr));
  }

  double tolerance_=1e-5;

  // Correct Linear sys
  EXP0_System linSys;
  std::unique_ptr<EXP0_SystemDerivative> linDeriv0;

  // Incorrect derivative for Linear sys
  std::unique_ptr<EXP1_SystemDerivative> linDeriv1;

  // Nonlinear system
  EXP2_System nonLinSys;
  std::unique_ptr<EXP2_SystemDerivative> nonLinDeriv;

  // Nonlinear system, linearised at \theta = \pi
  std::unique_ptr<EXP3_SystemDerivative> nonLinDerivUnstable;

};

TEST_F(SystemDynamicsLinearizerTest, testToleranceAndReturnTrue)
{
  auto state = EXP0_System::state_vector_t::Zero().eval();
  auto input = EXP0_System::input_vector_t::Zero().eval();
  EXP0_SystemDerivative::state_matrix_t A_error;
  EXP0_SystemDerivative::state_input_matrix_t B_error;
  bool ret = linDeriv0->derivativeChecker(0, state, input, A_error, B_error, tolerance_);
  auto numeric_error = std::fmax(A_error.template lpNorm<Eigen::Infinity>(), B_error.template lpNorm<Eigen::Infinity>());
  ASSERT_TRUE((numeric_error <= tolerance_) && (ret)) <<
                                                      "If the error is within the tolerance the checker should return true." <<
                                                      " tolerance: " << tolerance_ <<
                                                      " error: " << numeric_error <<
                                                      " returned: " << ret;
}

/**
 * Uses EXP0_System but with incorrect SystemDerivative (EXP1_SystemDerivative)
 */
TEST_F(SystemDynamicsLinearizerTest, testIncorrectLinearSystem)
{
  auto state = EXP0_System::state_vector_t::Random().eval();
  auto input = EXP0_System::input_vector_t::Random().eval();
  EXP1_SystemDerivative::state_matrix_t A_error;
  EXP1_SystemDerivative::state_input_matrix_t B_error;
  bool ret = linDeriv1->derivativeChecker(0, state, input, A_error, B_error, tolerance_);
  auto numeric_error = std::fmax(A_error.template lpNorm<Eigen::Infinity>(), B_error.template lpNorm<Eigen::Infinity>());
  ASSERT_TRUE((numeric_error > tolerance_) && (!ret)) <<
                           "If the error is within the tolerance the checker should return true." <<
                           " tolerance: " << tolerance_ <<
                           " error: " << numeric_error <<
                           " returned: " << ret;
}

/**
 * Pendulum system: EXP2_System
 */
TEST_F(SystemDynamicsLinearizerTest, testPendulum)
{
  const size_t divisions = 30;
  const double maxDeg = 180.0;
  constexpr double toRads = M_PI / 180.0;

  auto temp = Eigen::Vector2d(2);
  temp << 0, 1;
  Eigen::Matrix<double, 2, divisions> testStates;
  testStates.row(0).setZero();
  testStates.row(1).setLinSpaced(0, maxDeg);
  testStates.row(1) *= toRads;
  testStates.eval();
  auto input = EXP2_System::input_vector_t::Random().eval();

  for (auto i=0; i<divisions; ++i){
    EXP2_SystemDerivative::state_matrix_t A_error;
    EXP2_SystemDerivative::state_input_matrix_t B_error;
    bool ret = nonLinDeriv->derivativeChecker(0, testStates.col(i), input, A_error, B_error, tolerance_);
    auto numeric_error = std::fmax(A_error.template lpNorm<Eigen::Infinity>(), B_error.template lpNorm<Eigen::Infinity>());
    ASSERT_TRUE(((numeric_error <= tolerance_) && ret) || ((numeric_error > tolerance_ ) && !ret))
                << "If the error is within the tolerance the checker should return true."
                << " tolerance: " << tolerance_ << " error: " << numeric_error
                << " returned: " << ret;

    if (testStates(1,i) < M_PI_2 *0.5) {
      EXPECT_TRUE(ret)
              << "We expect the system to not have diverged too much yet\n"
                  << "index: " << i
              << " state: " << testStates.col(i)
              << " sin(theta): " << std::sin(testStates(1,i)) << "\n"
                  << "A_error: " << A_error ;
    }
    else {
      EXPECT_FALSE(ret)
          << "We expect the system to have diverged!\n"
              << "index: " << i
          << " sin(theta): " << std::sin(testStates(1,i))
          << " state: " << testStates.col(i) << "\n"
              << "A_error: " << A_error;
    }
  }
}

/**
 * Pendulum system: EXP3_System
 */
TEST_F(SystemDynamicsLinearizerTest, testPendulumUnstable)
{
  const size_t divisions = 30;
  const double maxDeg = 180.0;
  constexpr double toRads = M_PI / 180.0;

  auto temp = Eigen::Vector2d(2);
  temp << 0, 1;
  Eigen::Matrix<double, 2, divisions> testStates;
  testStates.row(0).setZero();
  testStates.row(1).setLinSpaced(0, maxDeg);
  testStates.row(1) *= toRads;
  testStates.eval();
  auto input = EXP2_System::input_vector_t::Random().eval();

  for (auto i=0; i<divisions; ++i){
    EXP3_SystemDerivative::state_matrix_t A_error;
    EXP3_SystemDerivative::state_input_matrix_t B_error;
    bool ret = nonLinDeriv->derivativeChecker(0, testStates.col(i), input, A_error, B_error, tolerance_);
    auto numeric_error = std::fmax(A_error.template lpNorm<Eigen::Infinity>(), B_error.template lpNorm<Eigen::Infinity>());
    ASSERT_TRUE(((numeric_error <= tolerance_) && ret) || ((numeric_error > tolerance_ ) && !ret))
                << "If the error is within the tolerance the checker should return true."
                << " tolerance: " << tolerance_ << " error: " << numeric_error
                << " returned: " << ret;

    if (testStates(1,i) > M_PI * 0.7) {
      EXPECT_TRUE(ret)
              << "We expect the system to not have diverged too much yet\n"
              << "index: " << i
              << " state: " << testStates.col(i)
              << " sin(theta): " << std::sin(testStates(1,i)) << "\n"
              << "A_error: " << A_error ;
    }
    else {
      EXPECT_FALSE(ret)
              << "We expect the system to have diverged!\n"
                  << "index: " << i
              << " sin(theta): " << std::sin(testStates(1,i))
              << " state: " << testStates.col(i) << "\n"
              << "A_error: " << A_error;
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

