/*
 * testDerivativeChecker.cpp
 *
 *  Created on: 2019-06-24
 *      Author: oharley
 */

#include "include/testSystemDynamicsLinearizer.h"
#include <Eigen/Dense>
#include "ocs2_core/automatic_differentiation/FiniteDifferenceMethods.h"
#include "ocs2_core/dynamics/SystemDynamicsLinearizer.h"

#include <gtest/gtest.h>

using namespace ocs2;

class SystemDynamicsLinearizerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // std::srand((unsigned int) time(0)); //Seed randomly
    std::srand((unsigned int)0xABCDEF);  // Seed repeatably

    /****************************
     *  The systems under test  *
     ****************************/

    EXP0_System linSys;
    EXP0_SystemDerivative linDeriv0;
    EXP1_SystemDerivative linDeriv1;  // purposefully Incorrect derivative for Linear sys
    linSys_ptr = std::make_shared<EXP0_System>();
    linDeriv0_ptr = std::make_shared<EXP0_SystemDerivative>();
    linDeriv1_ptr = std::make_shared<EXP1_SystemDerivative>();

    EXP2_System nonLinSys;
    EXP2_SystemDerivative nonLinDerivUnstable;  //! Nonlinear system, linearised at \theta = 0
    EXP3_SystemDerivative nonLinDeriv;
    nonLinSys_ptr = std::make_shared<EXP2_System>();
    // auto nonLinDeriv_ptr = std::make_shared<EXP2_SystemDerivative>();
    // auto nonLinDerivUnstable_ptr = std::make_shared<EXP3_SystemDerivative>();

    linDeriv0_ptr.reset(new EXP0_SystemDerivative());
    linDeriv1_ptr.reset(new EXP1_SystemDerivative());

    nonLinDerivUnstable_ptr.reset(new EXP2_SystemDerivative());
    nonLinDeriv_ptr.reset(new EXP3_SystemDerivative());

    /**********************************
     *  The FiniteDifference Checker  *
     **********************************/

    // the dimensions here are <2,1> for all systems but we leave it more general
    // linearizer = new SystemDynamicsLinearizer<EXP2_System::DIMENSIONS::DIMS::STATE_DIM_,
    // EXP2_System::DIMENSIONS::DIMS::INPUT_DIM_>(linSys_ptr);
  }

  using scalar_t = EXP0_System::scalar_t;
  scalar_t tolerance_ = 1e-5;

  // Correct Linear sys
  EXP0_System linSys;
  EXP0_SystemDerivative linDeriv0;
  EXP1_SystemDerivative linDeriv1;  // purposefully Incorrect derivative for Linear sys
  // Ptrs
  std::shared_ptr<EXP0_System> linSys_ptr;
  std::shared_ptr<EXP0_SystemDerivative> linDeriv0_ptr;
  std::shared_ptr<EXP1_SystemDerivative> linDeriv1_ptr;

  // Nonlinear system
  EXP2_System nonLinSys;
  EXP2_SystemDerivative nonLinDerivUnstable;  // linearised at \theta = 0, unstable
  EXP3_SystemDerivative nonLinDeriv;          // linearised at \theta = \pi, stable
  // Ptrs
  std::shared_ptr<EXP2_System> nonLinSys_ptr;
  std::shared_ptr<EXP2_SystemDerivative> nonLinDerivUnstable_ptr;
  std::shared_ptr<EXP3_SystemDerivative> nonLinDeriv_ptr;
};

/**
 * Uses EXP0_System
 */
TEST_F(SystemDynamicsLinearizerTest, testToleranceAndReturnTrue) {
  auto state = EXP0_System::state_vector_t::Zero().eval();
  auto input = EXP0_System::input_vector_t::Zero().eval();
  EXP0_SystemDerivative::state_matrix_t A_error;
  EXP0_SystemDerivative::state_input_matrix_t B_error;

  bool ret = FiniteDifferenceMethods<EXP0_System>::derivativeChecker(*linSys_ptr, *linDeriv0_ptr, tolerance_ * tolerance_, tolerance_, true,
                                                                     false, 0, state, input, A_error, B_error);

  auto numeric_error = std::fmax(A_error.template lpNorm<Eigen::Infinity>(), B_error.template lpNorm<Eigen::Infinity>());
  ASSERT_TRUE((numeric_error <= tolerance_) && (ret))
      << "If the error is within the tolerance the checker should return true."
      << " tolerance: " << tolerance_ << " error: " << numeric_error << " returned: " << ret;
}

/**
 * Uses EXP0_System but with incorrect SystemDerivative (EXP1_SystemDerivative)
 */
TEST_F(SystemDynamicsLinearizerTest, testIncorrectLinearSystem) {
  auto state = EXP0_System::state_vector_t::Random().eval();
  auto input = EXP0_System::input_vector_t::Random().eval();
  EXP1_SystemDerivative::state_matrix_t A_error;
  EXP1_SystemDerivative::state_input_matrix_t B_error;

  bool ret = FiniteDifferenceMethods<EXP0_System>::derivativeChecker(*linSys_ptr, *linDeriv1_ptr, tolerance_ * tolerance_, tolerance_, true,
                                                                     false, 0, state, input, A_error, B_error);
  auto numeric_error = std::fmax(A_error.template lpNorm<Eigen::Infinity>(), B_error.template lpNorm<Eigen::Infinity>());
  ASSERT_TRUE((numeric_error > tolerance_) && (!ret))
      << "If the error is within the tolerance the checker should return true."
      << " tolerance: " << tolerance_ << " error: " << numeric_error << " returned: " << ret;
}

/**
 * Pendulum system: EXP2_System
 */
TEST_F(SystemDynamicsLinearizerTest, testPendulumUnstable) {
  const size_t divisions = 1000;
  const scalar_t maxDeg = 180.0;
  constexpr scalar_t toRads = M_PI / 180.0;

  Eigen::Matrix<scalar_t, 2, divisions> testStates;
  testStates.row(0).setLinSpaced(0, maxDeg);  // initial starting points between upright and down
  testStates.row(0) *= toRads;
  testStates.row(1).setZero();  // Zero initial starting velocity
  testStates.eval();
  auto input = EXP2_System::input_vector_t::Random().eval();
  EXP2_SystemDerivative::state_matrix_t A_error;
  EXP2_SystemDerivative::state_input_matrix_t B_error;

  auto checker = FiniteDifferenceMethods<EXP2_System>(nonLinSys_ptr, tolerance_ * tolerance_, tolerance_);

  for (auto i = 0; i < divisions; ++i) {
    A_error.setZero();
    B_error.setZero();
    bool ret = checker.derivativeChecker(*nonLinDerivUnstable_ptr, 0, testStates.col(i), input, A_error, B_error);
    auto numeric_error = std::fmax(A_error.template lpNorm<Eigen::Infinity>(), B_error.template lpNorm<Eigen::Infinity>());
    ASSERT_TRUE(((numeric_error <= tolerance_) && ret) || ((numeric_error > tolerance_) && !ret))
        << "If the error is within the tolerance the checker should return true."
        << " tolerance: " << tolerance_ << " error: " << numeric_error << " returned: " << ret;

    // The 3*tolerance_ is chosen to make the tests pass, we wouldn't expect it to be much more for this simple system
    if (testStates(0, i) < std::sqrt(tolerance_)) {
      EXPECT_TRUE(ret) << "We expect the system to not have diverged too much yet\n"
                       << "index: " << i << "\n"
                       << "state: " << testStates.col(i) << "\n"
                       << "theta rad:" << testStates(0, i) << " deg: " << testStates(0, i) * 180.0 / M_PI << "\n"
                       << "sin(theta): " << std::sin(testStates(0, i)) << "\n"
                       << "A_error: " << A_error;
    } else {
      EXPECT_FALSE(ret) << "We expect the system to have diverged!\n"
                        << "index: " << i << "\n"
                        << "theta rad:" << testStates(0, i) << " deg: " << testStates(0, i) * 180.0 / M_PI << "\n"
                        << " sin(theta): " << std::sin(testStates(0, i)) << "\n"
                        << " state: " << testStates.col(i) << "\n"
                        << "A_error: " << A_error;
    }
  }
}

/**
 * Pendulum system: EXP3_System
 */
TEST_F(SystemDynamicsLinearizerTest, testPendulum) {
  const size_t divisions = 1000;
  const scalar_t maxDeg = 180.0;
  constexpr scalar_t toRads = M_PI / 180.0;

  Eigen::Matrix<scalar_t, 2, divisions> testStates;
  testStates.row(0).setLinSpaced(0, maxDeg);  // initial starting points between upright and down
  testStates.row(0) *= toRads;
  testStates.row(1).setZero();  // Zero initial starting velocity
  testStates.eval();
  auto input = EXP2_System::input_vector_t::Random().eval();
  FiniteDifferenceMethods<EXP2_System>::state_matrix_t A_error;
  FiniteDifferenceMethods<EXP2_System>::state_input_matrix_t B_error;
  auto checker = FiniteDifferenceMethods<EXP2_System>(nonLinSys_ptr, tolerance_ * tolerance_, tolerance_);

  // linearizer->setSystems(nonLinSys_ptr, nonLinDeriv_ptr);

  for (auto i = 0; i < divisions; ++i) {
    A_error.setZero();
    B_error.setZero();
    bool ret = checker.derivativeChecker(*nonLinDeriv_ptr, 0, testStates.col(i), input, A_error, B_error);
    auto numeric_error = std::fmax(A_error.template lpNorm<Eigen::Infinity>(), B_error.template lpNorm<Eigen::Infinity>());
    ASSERT_TRUE(((numeric_error <= tolerance_) && ret) || ((numeric_error > tolerance_) && !ret))
        << "If the error is within the tolerance the checker should return true."
        << " tolerance: " << tolerance_ << " error: " << numeric_error << " returned: " << ret;

    if (testStates(0, i) > M_PI - std::sqrt(tolerance_)) {
      EXPECT_TRUE(ret) << "We expect the system to not have diverged too much yet\n"
                       << "index: " << i << "\n"
                       << "theta rad:" << testStates(0, i) << " deg: " << testStates(0, i) * 180.0 / M_PI << "\n"
                       << " sin(theta): " << std::sin(testStates(0, i)) << "\n"
                       << "state: " << testStates.col(i) << "\n"
                       << "A_error: " << A_error;
    } else {
      EXPECT_FALSE(ret) << "We expect the system to have diverged!\n"
                        << "index: " << i << "theta rad:" << testStates(0, i) << " deg: " << testStates(0, i) * 180.0 / M_PI << "\n"
                        << " sin(theta): " << std::sin(testStates(0, i)) << " state: " << testStates.col(i) << "\n"
                        << "A_error: " << A_error;
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
