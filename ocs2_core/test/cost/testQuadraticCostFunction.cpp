#include <gtest/gtest.h>

#include <ocs2_core/cost/QuadraticCostFunction.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>

using namespace ocs2;

class testQuadraticCostFunction : public testing::Test {
 protected:
  testQuadraticCostFunction() {
    Q_ << 2, 1, 1, 2;
    Qf_ << 1, 0, 0, 1;
    R_ << 2;
    P_ << 1, 1;

    xNominal_.setRandom(2);
    uNominal_.setRandom(1);
    targetTrajectories_ = TargetTrajectories({0.0}, {xNominal_}, {uNominal_});

    x_.setRandom(2);
    u_.setRandom(1);

    vector_t dx = x_ - xNominal_;
    vector_t du = u_ - uNominal_;

    expectedCost_ = 0.5 * dx.dot(Q_ * dx) + 0.5 * du.dot(R_ * du) + du.dot(P_ * dx);
    expectedFinalCost_ = 0.5 * dx.dot(Qf_ * dx);
  }

  const scalar_t PRECISION = 1e-9;

  matrix_t Q_{2, 2};
  matrix_t Qf_{2, 2};
  matrix_t R_{1, 1};
  matrix_t P_{1, 2};
  scalar_t t_ = 0.0;
  vector_t x_;
  vector_t u_;
  vector_t xNominal_;
  vector_t uNominal_;
  TargetTrajectories targetTrajectories_;

  scalar_t expectedCost_;
  scalar_t expectedFinalCost_;
};

TEST_F(testQuadraticCostFunction, costValues) {
  QuadraticCostFunction costFunction(Q_, R_, Qf_, P_);
  costFunction.setTargetTrajectoriesPtr(&targetTrajectories_);

  EXPECT_NEAR(costFunction.cost(t_, x_, u_), expectedCost_, PRECISION);
  EXPECT_NEAR(costFunction.finalCost(t_, x_), expectedFinalCost_, PRECISION);
}

TEST_F(testQuadraticCostFunction, costApproximation) {
  QuadraticCostFunction costFunction(Q_, R_, Qf_, P_);
  costFunction.setTargetTrajectoriesPtr(&targetTrajectories_);

  auto L = costFunction.costQuadraticApproximation(t_, x_, u_);

  vector_t dx = x_ - xNominal_;
  vector_t du = u_ - uNominal_;
  EXPECT_NEAR(L.f, expectedCost_, PRECISION);
  EXPECT_TRUE(L.dfdx.isApprox(Q_ * dx + P_.transpose() * du, PRECISION));
  EXPECT_TRUE(L.dfdu.isApprox(R_ * du + P_ * dx, PRECISION));
  EXPECT_TRUE(L.dfdxx.isApprox(Q_, PRECISION));
  EXPECT_TRUE(L.dfdux.isApprox(P_, PRECISION));
  EXPECT_TRUE(L.dfduu.isApprox(R_, PRECISION));

  EXPECT_NEAR(L.f, costFunction.cost(t_, x_, u_), PRECISION);
}

TEST_F(testQuadraticCostFunction, finalCostApproximation) {
  QuadraticCostFunction costFunction(Q_, R_, Qf_, P_);
  costFunction.setTargetTrajectoriesPtr(&targetTrajectories_);

  auto Phi = costFunction.finalCostQuadraticApproximation(t_, x_);

  vector_t dx = x_ - xNominal_;
  EXPECT_NEAR(Phi.f, expectedFinalCost_, PRECISION);
  EXPECT_TRUE(Phi.dfdx.isApprox(Qf_ * dx, PRECISION));
  EXPECT_TRUE(Phi.dfdxx.isApprox(Qf_, PRECISION));

  EXPECT_NEAR(Phi.f, costFunction.finalCost(t_, x_), PRECISION);
}

TEST_F(testQuadraticCostFunction, StateInputCostValue) {
  QuadraticStateInputCost costFunction(Q_, R_, P_);

  auto L = costFunction.getValue(t_, x_, u_, targetTrajectories_);
  EXPECT_NEAR(L, expectedCost_, PRECISION);
}

TEST_F(testQuadraticCostFunction, StateInputCostApproximation) {
  QuadraticStateInputCost costFunction(Q_, R_, P_);

  auto L = costFunction.getQuadraticApproximation(t_, x_, u_, targetTrajectories_);

  vector_t dx = x_ - xNominal_;
  vector_t du = u_ - uNominal_;
  EXPECT_NEAR(L.f, expectedCost_, PRECISION);
  EXPECT_TRUE(L.dfdx.isApprox(Q_ * dx + P_.transpose() * du, PRECISION));
  EXPECT_TRUE(L.dfdu.isApprox(R_ * du + P_ * dx, PRECISION));
  EXPECT_TRUE(L.dfdxx.isApprox(Q_, PRECISION));
  EXPECT_TRUE(L.dfdux.isApprox(P_, PRECISION));
  EXPECT_TRUE(L.dfduu.isApprox(R_, PRECISION));
}

TEST_F(testQuadraticCostFunction, StateCostValue) {
  QuadraticStateCost costFunction(Qf_);

  auto L = costFunction.getValue(t_, x_, targetTrajectories_);
  EXPECT_NEAR(L, expectedFinalCost_, PRECISION);
}

TEST_F(testQuadraticCostFunction, StateCostApproximation) {
  QuadraticStateCost costFunction(Qf_);

  auto Phi = costFunction.getQuadraticApproximation(t_, x_, targetTrajectories_);

  vector_t dx = x_ - xNominal_;
  EXPECT_NEAR(Phi.f, expectedFinalCost_, PRECISION);
  EXPECT_TRUE(Phi.dfdx.isApprox(Qf_ * dx, PRECISION));
  EXPECT_TRUE(Phi.dfdxx.isApprox(Qf_, PRECISION));
}
