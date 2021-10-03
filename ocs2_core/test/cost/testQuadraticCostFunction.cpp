#include <gtest/gtest.h>

#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>

using namespace ocs2;

class testQuadraticCost : public testing::Test {
 protected:
  testQuadraticCost() {
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
  const ocs2::PreComputation preComputation_;

  scalar_t expectedCost_;
  scalar_t expectedFinalCost_;
};

TEST_F(testQuadraticCost, StateInputCostValue) {
  QuadraticStateInputCost costFunction(Q_, R_, P_);

  auto L = costFunction.getValue(t_, x_, u_, targetTrajectories_, preComputation_);
  EXPECT_NEAR(L, expectedCost_, PRECISION);
}

TEST_F(testQuadraticCost, StateInputCostApproximation) {
  QuadraticStateInputCost costFunction(Q_, R_, P_);

  auto L = costFunction.getQuadraticApproximation(t_, x_, u_, targetTrajectories_, preComputation_);

  vector_t dx = x_ - xNominal_;
  vector_t du = u_ - uNominal_;
  EXPECT_NEAR(L.f, expectedCost_, PRECISION);
  EXPECT_TRUE(L.dfdx.isApprox(Q_ * dx + P_.transpose() * du, PRECISION));
  EXPECT_TRUE(L.dfdu.isApprox(R_ * du + P_ * dx, PRECISION));
  EXPECT_TRUE(L.dfdxx.isApprox(Q_, PRECISION));
  EXPECT_TRUE(L.dfdux.isApprox(P_, PRECISION));
  EXPECT_TRUE(L.dfduu.isApprox(R_, PRECISION));
}

TEST_F(testQuadraticCost, StateInputCostClone) {
  QuadraticStateInputCost costFunction(Q_, R_, P_);
  auto costFunctionClone = std::unique_ptr<StateInputCost>(costFunction.clone());

  auto L = costFunction.getValue(t_, x_, u_, targetTrajectories_, preComputation_);
  auto Lclone = costFunctionClone->getValue(t_, x_, u_, targetTrajectories_, preComputation_);
  EXPECT_NEAR(L, Lclone, PRECISION);
}

TEST_F(testQuadraticCost, StateCostValue) {
  QuadraticStateCost costFunction(Qf_);

  auto L = costFunction.getValue(t_, x_, targetTrajectories_, preComputation_);
  EXPECT_NEAR(L, expectedFinalCost_, PRECISION);
}

TEST_F(testQuadraticCost, StateCostApproximation) {
  QuadraticStateCost costFunction(Qf_);

  auto Phi = costFunction.getQuadraticApproximation(t_, x_, targetTrajectories_, preComputation_);

  vector_t dx = x_ - xNominal_;
  EXPECT_NEAR(Phi.f, expectedFinalCost_, PRECISION);
  EXPECT_TRUE(Phi.dfdx.isApprox(Qf_ * dx, PRECISION));
  EXPECT_TRUE(Phi.dfdxx.isApprox(Qf_, PRECISION));
}

TEST_F(testQuadraticCost, StateCostClone) {
  QuadraticStateCost costFunction(Qf_);
  auto costFunctionClone = std::unique_ptr<StateCost>(costFunction.clone());

  auto L = costFunction.getValue(t_, x_, targetTrajectories_, preComputation_);
  auto Lclone = costFunctionClone->getValue(t_, x_, targetTrajectories_, preComputation_);
  EXPECT_NEAR(L, Lclone, PRECISION);
}
