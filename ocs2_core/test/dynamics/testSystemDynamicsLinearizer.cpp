#include <gtest/gtest.h>

#include <random>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/FiniteDifferenceMethods.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/dynamics/SystemDynamicsLinearizer.h>

using namespace ocs2;

static bool derivativeChecker(ControlledSystemBase& nonlinearSystem, DerivativesBase& derivatives, scalar_t eps, scalar_t tolerance,
                              bool doubleSidedDerivative, bool isSecondOrderSystem, scalar_t t, const vector_t& x, const vector_t& u,
                              matrix_t& A_error, matrix_t& B_error);

class EXP0_System : public ControlledSystemBase {
 public:
  EXP0_System() = default;
  ~EXP0_System() override = default;

  void computeFlowMap(const scalar_t& t, const vector_t& x, const vector_t& u, vector_t& dxdt) {
    Eigen::Matrix2d A;
    A << 0.6, 1.2, -0.8, 3.4;
    Eigen::Vector2d B;
    B << 1, 1;
    dxdt = A * x + B * u;
  }

  EXP0_System* clone() const final { return new EXP0_System(*this); }
};

class EXP0_SystemDerivative : public DerivativesBase {
 public:
  EXP0_SystemDerivative() = default;
  ~EXP0_SystemDerivative() override = default;

  void getFlowMapDerivativeState(matrix_t& A) final {
    A.resize(2, 2);
    A << 0.6, 1.2, -0.8, 3.4;
  }
  void getFlowMapDerivativeInput(matrix_t& B) final {
    B.resize(2, 1);
    B << 1, 1;
  }

  EXP0_SystemDerivative* clone() const final { return new EXP0_SystemDerivative(*this); }
};

/**
 * Has incorrect linearised dynamics (A=0, B=0) but the dynamics are non-zero.
 */
class EXP1_SystemDerivative : public DerivativesBase {
 public:
  EXP1_SystemDerivative() = default;
  ~EXP1_SystemDerivative() override = default;

  void getFlowMapDerivativeState(matrix_t& A) final {
    A.resize(2, 2);
    A << 0, 0, 0, 0;
  }
  void getFlowMapDerivativeInput(matrix_t& B) final {
    B.resize(2, 1);
    B << 0, 0;
  }

  EXP1_SystemDerivative* clone() const final { return new EXP1_SystemDerivative(*this); }
};

/**
 * Pendulum system, \fn$ \theta = 0 \fn$ is upright
 */
class EXP2_System : public ControlledSystemBase {
 public:
  EXP2_System() = default;
  ~EXP2_System() override = default;

  void computeFlowMap(const scalar_t& t, const vector_t& x, const vector_t& u, vector_t& dxdt) final {
    Eigen::Vector2d Ax;
    Ax << x(1), sin(x(0));
    Eigen::Vector2d B;
    B << 0, 0.1;  // just random values
    dxdt = Ax + B * u;
  }

  EXP2_System* clone() const final { return new EXP2_System(*this); }
};

/**
 * Pendulum system, linearised at \fn$ \theta = 0  \fn$, up and unstable
 */
class EXP2_SystemDerivative : public DerivativesBase {
 public:
  EXP2_SystemDerivative() = default;
  ~EXP2_SystemDerivative() override = default;

  //! Linearised at \fn$ \theta=0 \fn$
  void getFlowMapDerivativeState(matrix_t& A) final {
    A.resize(2, 2);
    A << 0, 1, 1, 0;
  }
  void getFlowMapDerivativeInput(matrix_t& B) final {
    B.resize(2, 1);
    B << 0, 0.1;
  }

  EXP2_SystemDerivative* clone() const final { return new EXP2_SystemDerivative(*this); }
};

/**
 * Pendulum system, linearised at \fn$ \theta = \pi  \fn$ - down / stable
 */
class EXP3_SystemDerivative : public DerivativesBase {
 public:
  EXP3_SystemDerivative() = default;
  ~EXP3_SystemDerivative() override = default;

  //! Linearised at \fn$ \theta=\pi \fn$
  void getFlowMapDerivativeState(matrix_t& A) final {
    A.resize(2, 2);
    A << 0, 1, -1, 0;
  }
  void getFlowMapDerivativeInput(matrix_t& B) final {
    B.resize(2, 1);
    B << 0, 0.1;
  }

  EXP3_SystemDerivative* clone() const final { return new EXP3_SystemDerivative(*this); }
};

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

    nonLinDerivUnstable_ptr.reset(new EXP2_SystemDerivative());
    nonLinDeriv_ptr.reset(new EXP3_SystemDerivative());
  }

  const scalar_t tolerance_ = 1e-5;
  const scalar_t eps_ = 1e-10;

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
  auto state = vector_t::Zero(2);
  auto input = vector_t::Zero(1);
  matrix_t A_error(2, 2);
  matrix_t B_error(2, 1);

  bool ret = derivativeChecker(*linSys_ptr, *linDeriv0_ptr, eps_, tolerance_, true, false, 0, state, input, A_error, B_error);

  auto numeric_error = std::fmax(A_error.lpNorm<Eigen::Infinity>(), B_error.lpNorm<Eigen::Infinity>());
  ASSERT_TRUE((numeric_error <= tolerance_) && (ret))
      << "If the error is within the tolerance the checker should return true."
      << " tolerance: " << tolerance_ << " error: " << numeric_error << " returned: " << ret;
}

/**
 * Uses EXP0_System but with incorrect SystemDerivative (EXP1_SystemDerivative)
 */
TEST_F(SystemDynamicsLinearizerTest, testIncorrectLinearSystem) {
  auto state = vector_t::Random(2);
  auto input = vector_t::Random(1);
  matrix_t A_error(2, 2);
  matrix_t B_error(2, 1);

  bool ret = derivativeChecker(*linSys_ptr, *linDeriv1_ptr, eps_, tolerance_, true, false, 0, state, input, A_error, B_error);
  auto numeric_error = std::fmax(A_error.lpNorm<Eigen::Infinity>(), B_error.lpNorm<Eigen::Infinity>());
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
  const scalar_t t = 0;
  auto u = vector_t::Random(1);

  matrix_t testStates(2, divisions);
  testStates.row(0).setLinSpaced(0, toRads * maxDeg);  // initial starting points between upright and down
  testStates.row(1).setZero();                         // Zero initial starting velocity

  matrix_t A_error(2, 2);
  matrix_t B_error(2, 1);

  for (auto i = 0; i < divisions; ++i) {
    A_error.setZero(2, 2);
    B_error.setZero(2, 1);
    bool ret = derivativeChecker(*nonLinSys_ptr, *nonLinDerivUnstable_ptr, eps_, tolerance_, true, false, t, testStates.col(i), u, A_error,
                                 B_error);

    auto numeric_error = std::fmax(A_error.lpNorm<Eigen::Infinity>(), B_error.lpNorm<Eigen::Infinity>());

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
  const scalar_t t = 0;
  auto u = vector_t::Random(1);

  matrix_t testStates(2, divisions);
  testStates.row(0).setLinSpaced(0, toRads * maxDeg);  // initial starting points between upright and down
  testStates.row(1).setZero();                         // Zero initial starting velocity
  matrix_t A_error(2, 2);
  matrix_t B_error(2, 1);

  for (auto i = 0; i < divisions; ++i) {
    A_error.setZero();
    B_error.setZero();
    bool ret =
        derivativeChecker(*nonLinSys_ptr, *nonLinDeriv_ptr, eps_, tolerance_, true, false, t, testStates.col(i), u, A_error, B_error);
    auto numeric_error = std::fmax(A_error.lpNorm<Eigen::Infinity>(), B_error.lpNorm<Eigen::Infinity>());
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

static bool derivativeChecker(ControlledSystemBase& nonlinearSystem, DerivativesBase& derivatives, scalar_t eps, scalar_t tolerance,
                              bool doubleSidedDerivative, bool isSecondOrderSystem, scalar_t t, const vector_t& x, const vector_t& u,
                              matrix_t& A_error, matrix_t& B_error) {
  matrix_t A;
  matrix_t B;
  derivatives.setCurrentStateAndControl(t, x, u);
  derivatives.getFlowMapDerivativeState(A);
  derivatives.getFlowMapDerivativeInput(B);

  A_error = finiteDifferenceDerivativeState(nonlinearSystem, t, x, u, eps, doubleSidedDerivative, isSecondOrderSystem);
  B_error = finiteDifferenceDerivativeInput(nonlinearSystem, t, x, u, eps, doubleSidedDerivative, isSecondOrderSystem);

  A_error -= A;
  B_error -= B;
  return tolerance > std::fmax(A_error.lpNorm<Eigen::Infinity>(), B_error.lpNorm<Eigen::Infinity>());
}
