#include <gtest/gtest.h>
#include <cstdlib>
#include <ctime>
#include <iostream>

#include <ocs2_oc/rollout/StateTriggeredRollout.h>
#include <ocs2_oc/test/dynamics_hybrid_slq_test.h>

#include <ocs2_core/cost/QuadraticCostFunction.h>

#include <ocs2_ddp/SLQ.h>
#include <ocs2_ddp/SLQ_Settings.h>

/*
 * Test for StateTriggeredRollout in combination with SLQ
 *
 * The system being tested is a combination of two Linear system
 * it switches between these systems on an event
 *
 * Guard Surfaces are:   x[0]*x[1] < 0 when in mode 0
 *                       x[0]*x[1] > 0 when in mode 1
 *
 * Cost function is:     x(t)^T Q x(t) + u(t)^T R u(t) + x(t1)^T P x(t1)^T
 *                       Q = P = [50,0;0,50];
 *                       R = 1;
 *
 * Constraints are:      |u|< 2
 *                       |x[0]| < 2
 *
 * The following tests are implemented and performed:
 * (1) No penetration of Guard Surfaces
 * (2) Constraint compliance
 * (3) Check of cost function compared against cost calculated during trusted run of SLQ
 */
TEST(testStateRollOut_SLQ, HybridSystemSLQTest) {
  using namespace ocs2;

  SLQ_Settings slqSettings;
  slqSettings.useNominalTimeForBackwardPass_ = true;
  slqSettings.ddpSettings_.displayInfo_ = false;
  slqSettings.ddpSettings_.displayShortSummary_ = false;
  slqSettings.ddpSettings_.maxNumIterations_ = 30;
  slqSettings.ddpSettings_.nThreads_ = 1;
  slqSettings.ddpSettings_.stateConstraintPenaltyCoeff_ = 1.0;
  slqSettings.ddpSettings_.inequalityConstraintMu_ = 0.1;
  slqSettings.ddpSettings_.inequalityConstraintDelta_ = 1e-6;
  slqSettings.ddpSettings_.checkNumericalStability_ = false;
  slqSettings.ddpSettings_.absTolODE_ = 1e-10;
  slqSettings.ddpSettings_.relTolODE_ = 1e-7;
  slqSettings.ddpSettings_.maxNumStepsPerSecond_ = 1e5;
  slqSettings.ddpSettings_.useFeedbackPolicy_ = true;
  slqSettings.ddpSettings_.debugPrintRollout_ = false;

  Rollout_Settings rolloutSettings;
  rolloutSettings.absTolODE_ = 1e-10;
  rolloutSettings.relTolODE_ = 1e-7;
  rolloutSettings.maxNumStepsPerSecond_ = 1e5;

  double startTime = 0.0;
  double finalTime = 5.0;

  std::vector<double> partitioningTimes;
  partitioningTimes.push_back(startTime);
  partitioningTimes.push_back(finalTime);

  Eigen::Matrix<double, STATE_DIM, 1> initState = {0, 1, 1};

  // rollout
  hybridSysDynamics systemDynamics;
  StateTriggeredRollout<STATE_DIM, 1> stateTriggeredRollout(systemDynamics, rolloutSettings);

  // derivatives
  hybridSysDerivatives systemDerivatives;
  // constraints
  hybridSysConstraints systemConstraints;
  // cost function
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q;
  Q << 50, 0, 0, 0, 50, 0, 0, 0, 0;
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> P;
  P << 50, 0, 0, 0, 50, 0, 0, 0, 0;
  Eigen::Matrix<double, INPUT_DIM, INPUT_DIM> R;
  R << 1;
  Eigen::Matrix<double, INPUT_DIM, STATE_DIM> crossTerm;
  crossTerm.setZero();
  Eigen::Matrix<double, STATE_DIM, 1> xNominal;
  xNominal.setZero();
  Eigen::Matrix<double, INPUT_DIM, 1> uNominal;
  uNominal.setZero();

  QuadraticCostFunction<STATE_DIM, INPUT_DIM> systemCost(Q, R, xNominal, uNominal, P, xNominal, crossTerm);

  // operatingTrajectories
  Eigen::Matrix<double, STATE_DIM, 1> stateOperatingPoint = Eigen::Matrix<double, STATE_DIM, 1>::Zero();
  Eigen::Matrix<double, INPUT_DIM, 1> inputOperatingPoint = Eigen::Matrix<double, INPUT_DIM, 1>::Zero();
  system_op operatingTrajectories(stateOperatingPoint, inputOperatingPoint);

  std::cout << "Starting SLQ Procedure" << std::endl;
  // SLQ
  SLQ<STATE_DIM, INPUT_DIM> slqST(&stateTriggeredRollout, &systemDerivatives, &systemConstraints, &systemCost, &operatingTrajectories,
                                  slqSettings);
  slqST.run(startTime, initState, finalTime, partitioningTimes);
  SLQ<STATE_DIM, INPUT_DIM>::primal_solution_t solution = slqST.primalSolution(finalTime);
  std::cout << "SLQ Procedure Done" << std::endl;

  if (false) {
    for (int i = 0; i < solution.stateTrajectory_.size(); i++) {
      std::cout << i << ";" << solution.timeTrajectory_[i] << ";" << solution.stateTrajectory_[i][0] << ";"
                << solution.stateTrajectory_[i][1] << ";" << solution.stateTrajectory_[i][2] << ";" << solution.inputTrajectory_[i]
                << std::endl;
    }
  }

  double cost;
  for (int i = 0; i < solution.stateTrajectory_.size(); i++) {
    // Test 1 : Constraint Compliance
    double constraint0 = -solution.inputTrajectory_[i][0] + 2;
    double constraint1 = solution.inputTrajectory_[i][0] + 2;
    double constraint2 = solution.stateTrajectory_[i][0] + 2;
    double constraint3 = -solution.stateTrajectory_[i][0] + 2;

    EXPECT_GT(constraint0, 0);
    EXPECT_GT(constraint1, 0);
    EXPECT_GT(constraint2, 0);
    EXPECT_GT(constraint3, 0);

    // Test 2 : No penetration of guardSurfaces
    Eigen::VectorXd guardSurfacesValue;
    systemDynamics.computeGuardSurfaces(solution.timeTrajectory_[i], solution.stateTrajectory_[i], guardSurfacesValue);

    EXPECT_GT(guardSurfacesValue[0], -1e-10);
    if (!(guardSurfacesValue[0] > -1e-10)) {
      std::cout << solution.timeTrajectory_[i] << "," << guardSurfacesValue[0] << "," << guardSurfacesValue[1] << std::endl;
    }

    EXPECT_GT(guardSurfacesValue[1], -1e-10);
    if (!(guardSurfacesValue[1] > -1e-10)) {
      std::cout << solution.timeTrajectory_[i] << "," << guardSurfacesValue[0] << "," << guardSurfacesValue[1] << std::endl;
    }
  }
  // Test 3: Check of cost function
  auto performanceIndecesST = slqST.getPerformanceIndeces();
  EXPECT_LT(std::fabs(performanceIndecesST.merit - 18.938001), 1e-6);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
