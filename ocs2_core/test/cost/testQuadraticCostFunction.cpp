#include <gtest/gtest.h>

#include <ocs2_core/cost/QuadraticCostFunction.h>

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
    xNominalFinal_.setRandom(2);

    x_.setRandom(2);
    u_.setRandom(1);

    vector_t dx = x_ - xNominal_;
    vector_t du = u_ - uNominal_;
    vector_t dxf = x_ - xNominalFinal_;

    expectedCost_ = 0.5 * dx.dot(Q_ * dx) + 0.5 * du.dot(R_ * du) + du.dot(P_ * dx);
    expectedFinalCost_ = 0.5 * dxf.dot(Qf_ * dxf);
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
  vector_t xNominalFinal_;
  scalar_t expectedCost_;
  scalar_t expectedFinalCost_;
};

TEST_F(testQuadraticCostFunction, costValues) {
  QuadraticCostFunction costFunction(Q_, R_, xNominal_, uNominal_, Qf_, xNominalFinal_, P_);

  EXPECT_NEAR(costFunction.cost(t_, x_, u_), expectedCost_, PRECISION);
  EXPECT_NEAR(costFunction.finalCost(t_, x_), expectedFinalCost_, PRECISION);
}

TEST_F(testQuadraticCostFunction, costApproximation) {
  QuadraticCostFunction costFunction(Q_, R_, xNominal_, uNominal_, Qf_, xNominalFinal_, P_);

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
  QuadraticCostFunction costFunction(Q_, R_, xNominal_, uNominal_, Qf_, xNominalFinal_, P_);

  auto Phi = costFunction.finalCostQuadraticApproximation(t_, x_);

  vector_t dxf = x_ - xNominalFinal_;
  EXPECT_NEAR(Phi.f, expectedFinalCost_, PRECISION);
  EXPECT_TRUE(Phi.dfdx.isApprox(Qf_ * dxf, PRECISION));
  EXPECT_TRUE(Phi.dfdxx.isApprox(Qf_, PRECISION));

  EXPECT_NEAR(Phi.f, costFunction.finalCost(t_, x_), PRECISION);
}

TEST_F(testQuadraticCostFunction, usesCostDesiredTrajectories) {
  /** Pass zeros as nominal trajectory to ensure testing the right code path */
  QuadraticCostFunction costFunction(Q_, R_, vector_t::Zero(2), vector_t::Zero(1), Qf_, vector_t::Zero(2), P_);

  CostDesiredTrajectories traj({0.0}, {xNominal_}, {uNominal_});

  costFunction.setCostDesiredTrajectoriesPtr(&traj);
  EXPECT_NEAR(costFunction.cost(t_, x_, u_), expectedCost_, PRECISION);

  traj.desiredStateTrajectory()[0] = xNominalFinal_;
  EXPECT_NEAR(costFunction.finalCost(t_, x_), expectedFinalCost_, PRECISION);
}

class CostNominalTrajectoryOverride : public QuadraticCostFunction {
 public:
  /** Passes zeros as nominal trajectory to base class */
  CostNominalTrajectoryOverride(const matrix_t& Q, const matrix_t& R, const vector_t& xNominal, const vector_t& uNominal,
                                const matrix_t& QFinal, const vector_t& xNominalFinal, const matrix_t& P = matrix_t())
      : QuadraticCostFunction(Q, R, vector_t::Zero(2), vector_t::Zero(1), QFinal, vector_t::Zero(2), P),
        xNominalOverride_(xNominal),
        uNominalOverride_(uNominal),
        xNominalFinalOverride_(xNominalFinal) {}

  /** Override nominal trajectory */
  std::pair<vector_t, vector_t> getNominalStateInput(scalar_t t, const vector_t& x, const vector_t& u) final {
    return {xNominalOverride_, uNominalOverride_};
  }

  /** Override nominal trajectory */
  vector_t getNominalFinalState(scalar_t t, const vector_t& x) final { return xNominalFinalOverride_; }

  vector_t xNominalOverride_;
  vector_t uNominalOverride_;
  vector_t xNominalFinalOverride_;
};

TEST_F(testQuadraticCostFunction, overrideNominalTrajectory) {
  CostNominalTrajectoryOverride costFunction(Q_, R_, xNominal_, uNominal_, Qf_, xNominalFinal_, P_);

  EXPECT_NEAR(costFunction.cost(t_, x_, u_), expectedCost_, PRECISION);
  EXPECT_NEAR(costFunction.finalCost(t_, x_), expectedFinalCost_, PRECISION);
}
