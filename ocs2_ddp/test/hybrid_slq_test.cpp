#include <gtest/gtest.h>
#include <cstdlib>
#include <ctime>
#include <iostream>

#include <ocs2_oc/rollout/StateTriggeredRollout.h>
#include <ocs2_oc/test/dynamics_hybrid_slq_test.h>

#include <ocs2_core/cost/QuadraticCostFunction.h>

#include <ocs2_ddp/SLQ.h>
#include <ocs2_ddp/SLQ_Settings.h>

TEST(testStateRollOut_SLQ, RunExample) {
  using namespace ocs2;

  SLQ_Settings slqSettings;
  slqSettings.useNominalTimeForBackwardPass_ = true;
  slqSettings.ddpSettings_.displayInfo_ = false;
  slqSettings.ddpSettings_.displayShortSummary_ = true;
  slqSettings.ddpSettings_.maxNumIterations_ = 30;
  slqSettings.ddpSettings_.nThreads_ = 1;
  slqSettings.ddpSettings_.noStateConstraints_ = false;
  slqSettings.ddpSettings_.stateConstraintPenaltyCoeff_ = 1.0;
  slqSettings.ddpSettings_.inequalityConstraintMu_ = 0.1;
  slqSettings.ddpSettings_.inequalityConstraintDelta_ = 1e-6;
  slqSettings.ddpSettings_.checkNumericalStability_ = false;
  slqSettings.ddpSettings_.absTolODE_ = 1e-10;
  slqSettings.ddpSettings_.relTolODE_ = 1e-7;
  slqSettings.ddpSettings_.maxNumStepsPerSecond_ = 1e5;
  slqSettings.ddpSettings_.useFeedbackPolicy_ = false;
  slqSettings.ddpSettings_.debugPrintRollout_ = false;

  Rollout_Settings rolloutSettings;
  rolloutSettings.absTolODE_ = 1e-10;
  rolloutSettings.relTolODE_ = 1e-7;
  rolloutSettings.maxNumStepsPerSecond_ = 1e5;

  std::vector<double> eventTimes(0);
  std::vector<size_t> subsystemsSequence{1};

  std::shared_ptr<hybridSysLogic> logicRulesPtr(new hybridSysLogic(eventTimes, subsystemsSequence));

  double startTime = 0.0;
  double finalTime = 5.0;

  std::vector<double> partitioningTimes;
  partitioningTimes.push_back(startTime);
  partitioningTimes.push_back(finalTime);

  Eigen::Matrix<double, STATE_DIM, 1> initState = {5,2,1};

  // rollout
  hybridSysDynamics systemDynamics;
  StateTriggeredRollout<STATE_DIM, 1> stateTriggeredRollout(systemDynamics, rolloutSettings);

  // derivatives
  hybridSysDerivatives systemDerivatives;
  // constraints
  hybridSysConstraints systemConstraints;
  // cost function
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q;
  Q<< 0.5, 0, 0, 0, 0.5, 0, 0, 0, 0;
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> P;
  P<< 0.5, 0, 0, 0, 0.5, 0, 0, 0, 0;
  Eigen::Matrix<double, INPUT_DIM, INPUT_DIM> R;
  R<< 0.005;
  Eigen::Matrix<double, INPUT_DIM, STATE_DIM> crossTerm;
  crossTerm.setZero();
  Eigen::Matrix<double, STATE_DIM, 1> xNominal;
  xNominal.setZero();
  Eigen::Matrix<double, INPUT_DIM, 1> uNominal;
  uNominal.setZero();

  QuadraticCostFunction<STATE_DIM,INPUT_DIM> systemCost(Q,R,xNominal,uNominal,P,xNominal,crossTerm);

  // operatingTrajectories
  Eigen::Matrix<double, STATE_DIM, 1> stateOperatingPoint = Eigen::Matrix<double, STATE_DIM, 1>::Zero();
  Eigen::Matrix<double, INPUT_DIM, 1> inputOperatingPoint = Eigen::Matrix<double, INPUT_DIM, 1>::Zero();
  system_op operatingTrajectories(stateOperatingPoint, inputOperatingPoint);

  std::cout << "Starting SLQ Procedure" << std::endl;
  // SLQ
  SLQ<STATE_DIM, INPUT_DIM> slqST(&stateTriggeredRollout, &systemDerivatives, &systemConstraints, &systemCost, &operatingTrajectories, slqSettings,
                                  logicRulesPtr);
  slqST.run(startTime, initState, finalTime, partitioningTimes);
  SLQ<STATE_DIM, INPUT_DIM>::primal_solution_t solutionST = slqST.primalSolution(finalTime);
  std::cout << "SLQ Procedure Done" << std::endl;

  if (false) {
    for (int i = 0; i < solutionST.stateTrajectory_.size(); i++) {
      std::cout << i << ";" << solutionST.timeTrajectory_[i] << ";" << solutionST.stateTrajectory_[i][0] << ";"
                << solutionST.stateTrajectory_[i][1] << ";" << solutionST.stateTrajectory_[i][2] << ";" << solutionST.inputTrajectory_[i]
                << std::endl;
    }
  }

  for (int i = 0; i < solutionST.stateTrajectory_.size(); i++) {
    Eigen::VectorXd guardSurfacesValue;
    systemDynamics.computeGuardSurfaces(solutionST.timeTrajectory_[i], solutionST.stateTrajectory_[i], guardSurfacesValue);

    EXPECT_GT(guardSurfacesValue[0], -1e-10);

    if (!(guardSurfacesValue[0] > -1e-10)) {
      std::cout << solutionST.timeTrajectory_[i] << "," << guardSurfacesValue[0] << "," << guardSurfacesValue[1] << std::endl;
    }

    EXPECT_GT(guardSurfacesValue[1], -1e-10);
    if (!(guardSurfacesValue[1] > -1e-10)) {
      std::cout << solutionST.timeTrajectory_[i] << "," << guardSurfacesValue[0] << "," << guardSurfacesValue[1] << std::endl;
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
