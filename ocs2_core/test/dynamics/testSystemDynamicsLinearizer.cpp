#include <gtest/gtest.h>

#include <memory>
#include <random>

#include <ocs2_core/Types.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>
#include <ocs2_core/dynamics/SystemDynamicsLinearizer.h>

using namespace ocs2;

const scalar_t TOLERANCE = 1e-5;
const scalar_t EPSILON = 1e-10;

static bool derivativeChecker(SystemDynamicsBase& nonlinearSystem, SystemDynamicsBase& reference, scalar_t tolerance, scalar_t t,
                              const vector_t& x, const vector_t& u);

/**
 * Pendulum system, \fn$ \theta = 0 \fn$ is upright
 */
class PendulumSystem final : public SystemDynamicsBase {
 public:
  PendulumSystem() : SystemDynamicsBase() {}
  ~PendulumSystem() override = default;
  PendulumSystem* clone() const override { return new PendulumSystem(*this); }

  vector_t computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) override {
    vector_t dfdt(2);
    dfdt << x(1), sin(x(0)) + 0.1 * u(0);
    return dfdt;
  }

  VectorFunctionLinearApproximation linearApproximation(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) override {
    VectorFunctionLinearApproximation linearDynamics;
    linearDynamics.f = computeFlowMap(t, x, u, PreComputation());
    linearDynamics.dfdx.resize(2, 2);
    linearDynamics.dfdx << 0, 1,  // clang-format off
                           cos(x(0)), 0;  // clang-format on
    linearDynamics.dfdu.resize(2, 1);
    linearDynamics.dfdu << 0, 0.1;
    return linearDynamics;
  }
};

TEST(testSystemDynamicsLinearizer, testDerivativeChecker) {
  scalar_t time = 0.0;
  vector_t state = vector_t::Zero(2);
  vector_t input = vector_t::Zero(1);

  matrix_t A(2, 2);
  matrix_t B(2, 1);

  A << 0.6, 1.2, -0.8, 3.4;
  B << 1, 1;
  LinearSystemDynamics linSys(A, B);

  A(0, 0) = 0;
  B(0, 0) = 0;
  LinearSystemDynamics alteredSys(A, B);

  ASSERT_FALSE(derivativeChecker(linSys, alteredSys, TOLERANCE, time, state, input));
}

TEST(testSystemDynamicsLinearizer, testLinearSystem) {
  scalar_t time = 0.0;
  vector_t state = vector_t::Zero(2);
  vector_t input = vector_t::Zero(1);

  matrix_t A(2, 2);
  A << 0.6, 1.2, -0.8, 3.4;
  matrix_t B(2, 1);
  B << 1, 1;
  LinearSystemDynamics linSys(A, B);

  SystemDynamicsLinearizer linearizedSys(std::unique_ptr<ControlledSystemBase>(linSys.clone()), /*doubleSidedDerivative=*/true,
                                         /*isSecondOrderSystem=*/false, EPSILON);

  ASSERT_TRUE(derivativeChecker(linSys, linearizedSys, TOLERANCE, time, state, input));
}

TEST(testSystemDynamicsLinearizer, testPendulum) {
  std::srand((unsigned int)0);  // Seed repeatably

  const size_t divisions = 1000;
  const scalar_t maxDeg = 180.0;
  constexpr scalar_t toRads = M_PI / 180.0;
  const scalar_t t = 0;

  matrix_t testStates(2, divisions);
  testStates.row(0).setLinSpaced(0, toRads * maxDeg);  // initial starting points between upright and down
  testStates.row(1).setZero();                         // Zero initial starting velocity
  vector_t input = vector_t::Random(1);

  PendulumSystem nonLinSys;
  SystemDynamicsLinearizer linearizedSys(std::unique_ptr<ControlledSystemBase>(nonLinSys.clone()), /*doubleSidedDerivative=*/true,
                                         /*isSecondOrderSystem=*/false, EPSILON);

  for (auto i = 0; i < divisions; ++i) {
    ASSERT_TRUE(derivativeChecker(nonLinSys, linearizedSys, TOLERANCE, t, testStates.col(i), input));
  }
}

static bool derivativeChecker(SystemDynamicsBase& sys1, SystemDynamicsBase& sys2, scalar_t tolerance, scalar_t t, const vector_t& x,
                              const vector_t& u) {
  auto derivatives1 = sys1.linearApproximation(t, x, u, PreComputation());
  auto derivatives2 = sys2.linearApproximation(t, x, u, PreComputation());
  scalar_t A_error = (derivatives1.dfdx - derivatives2.dfdx).lpNorm<Eigen::Infinity>();
  scalar_t B_error = (derivatives1.dfdu - derivatives2.dfdu).lpNorm<Eigen::Infinity>();
  return tolerance > std::fmax(A_error, B_error);
}
