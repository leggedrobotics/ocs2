#include <gtest/gtest.h>
#include <cstdlib>
#include <ctime>
#include <iostream>

#include <ocs2_oc/rollout/StateTriggeredRollout.h>

#include <ocs2_slq/SLQ.h>
#include <ocs2_slq/SLQ_MP.h>
#include <ocs2_slq/SLQ_Settings.h>

#include <ocs2_oc/test/Dynamics_StateRollOut_SLQ.h>

using namespace ocs2;

int main()
{


SLQ_Settings slqSettings;
  slqSettings.useNominalTimeForBackwardPass_ = true;
  slqSettings.ddpSettings_.displayInfo_ = true;
  slqSettings.ddpSettings_.displayShortSummary_ = false;
  slqSettings.ddpSettings_.maxNumIterations_ = 30;
  slqSettings.ddpSettings_.nThreads_ = 1;
  slqSettings.ddpSettings_.maxNumIterations_ = 30;
  slqSettings.ddpSettings_.lsStepsizeGreedy_ = true;
  slqSettings.ddpSettings_.noStateConstraints_ = true;
  slqSettings.ddpSettings_.checkNumericalStability_ = true;
  slqSettings.ddpSettings_.absTolODE_ = 1e-3;
  slqSettings.ddpSettings_.relTolODE_ = 1e-7;
  slqSettings.ddpSettings_.maxNumStepsPerSecond_ = 1e6;
  slqSettings.ddpSettings_.useFeedbackPolicy_ = false;
  slqSettings.ddpSettings_.debugPrintRollout_ = false;

Rollout_Settings rolloutSettings;
  rolloutSettings.absTolODE_ = 1e-3;
  rolloutSettings.relTolODE_ = 1e-7;
  rolloutSettings.maxNumStepsPerSecond_ = 1e6;

std::vector<double> eventTimes(0);
std::vector<size_t> subsystemsSequence{0};

std::shared_ptr<ball_tester_logic> logicRules(new ball_tester_logic(eventTimes,subsystemsSequence));

double startTime = 0.0;
double finalTime = 5;

std::vector<double> partitioningTimes;
partitioningTimes.push_back(startTime);
partitioningTimes.push_back(finalTime);

ball_tester_dyn::state_vector_t initState(1.0,1.0,0.0);

/*****												
*****/

// rollout
ball_tester_dyn sysdyn;
StateTriggeredRollout<3,1> stateTriggeredRollout(sysdyn,rolloutSettings);

// derivatives
ball_tester_der sysder;
// constraints
ball_tester_constr sysconstr;
// cost function
ball_tester_cost syscost;

// operatingTrajectories
  Eigen::Matrix<double, 3, 1> stateOperatingPoint = Eigen::Matrix<double, 3, 1>::Zero();
  Eigen::Matrix<double, 1, 1> inputOperatingPoint = Eigen::Matrix<double, 1, 1>::Zero();
  ball_tester_op operatingTrajectories(stateOperatingPoint, inputOperatingPoint);

// SLQ 
SLQ<3,1> slqST(&stateTriggeredRollout, &sysder, &sysconstr, &syscost, &operatingTrajectories,slqSettings,logicRules);

slqST.run(startTime, initState, finalTime, partitioningTimes);

SLQ_BASE<3, 1>::primal_solution_t solutionST = slqST.primalSolution(finalTime);

for(int i = 0; i<solutionST.stateTrajectory_.size();i++)
{
	std::cout<<i<<";"<<solutionST.timeTrajectory_[i]<<";"<<solutionST.stateTrajectory_[i][0]<<";"<<solutionST.stateTrajectory_[i][1]<<";"<<solutionST.stateTrajectory_[i][2]<<";"<<solutionST.inputTrajectory_[i]<<std::endl;
}



return 0;
}
