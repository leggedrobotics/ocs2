#include <iostream>
#include <OverallReference.h>

#include "SystemModel.h"

#include <ocs2_oc/rollout/StateTriggeredRollout.h>
#include <ocs2_core/control/StateBasedLinearController.h>

int main()
{
	using dynamic_vector_t = Eigen::Matrix<double, Eigen::Dynamic, 1>;
	using dynamic_vector_array_t = std::vector<dynamic_vector_t, Eigen::aligned_allocator<dynamic_vector_t>>;

	using state_control_t = ocs2::stateBasedLinearController<STATE_DIM,INPUT_DIM>;

	ocs2::SLQ_Settings slqSettings;
	slqSettings.useNominalTimeForBackwardPass_ = true;
	slqSettings.ddpSettings_.displayInfo_ = true;
	slqSettings.ddpSettings_.displayShortSummary_ = true;
	slqSettings.ddpSettings_.maxNumIterations_ = 30;
	slqSettings.ddpSettings_.nThreads_ = 1;
	slqSettings.ddpSettings_.noStateConstraints_ = true;
	slqSettings.ddpSettings_.checkNumericalStability_ = true;
	slqSettings.ddpSettings_.absTolODE_ = 1e-10;
	slqSettings.ddpSettings_.relTolODE_ = 1e-7;
	slqSettings.ddpSettings_.maxNumStepsPerSecond_ = 1e7;
	slqSettings.ddpSettings_.useFeedbackPolicy_ = true;
	slqSettings.ddpSettings_.debugPrintRollout_ = false;

	ocs2::Rollout_Settings rolloutSettings;
	rolloutSettings.absTolODE_ = 1e-10;
	rolloutSettings.relTolODE_ = 1e-7;
	rolloutSettings.maxNumStepsPerSecond_ = 1e7;
	rolloutSettings.maxSingleEventIterations_ = 10;

	// Parameters
	const double startTime = 0.0;
	const double finalTime = 2.5;
	const state_vector_t x0 = {0.7,0.5,0};

	// Generation of Reference Trajectory
	const std::vector<double> trajTimes{0,0.2,0.8,1.0,1.2,1.8,2.0};

	Eigen::Vector3d state0;		//	Intial and final state
	state0 << 0.5, 0, 0;
	Eigen::Vector3d state1;		//	Hard impact
	state1 << 0, -5, 0;
	Eigen::Vector3d state2;		// 	Soft impact
	state2 << 0, -1, 0;

	const scalar_t delta = 0.5;
	const std::vector<Eigen::Vector3d> trajStates{state0,state1,state2,state2,state1,state2,state0};
	OverallReference reference(trajTimes,trajStates);
	reference.extendref(delta);

	reference.display(0);

	scalar_array_t timeRef;
	scalar_array_t inputRef;
	reference.getInput(startTime,finalTime,0.01,timeRef,inputRef);
	// Dynamics, Constraints and derivative classes
	systemDynamics systemModel(reference,false);
	systemDerivative systemDerivatives;
	systemConstraint systemConstraints;

	// Cost Function
	state_matrix_t Q;
	Q<< 	50.0, 0.0,  0.0,
			0.0,  50.0, 0.0,
			0.0,  0.0,  0.0;

	input_matrix_t R(1);
	state_matrix_t P;
	P << 	56.63, 7.07, 0.0,
			7.07,  8.01, 0.0,
			0.0,   0.0,  0.0;

	state_vector_t xNom = state0;
	input_vector_t uNom(0);
	state_vector_t xFin = state0;
	systemCost systemCost(reference,Q,R,P,xNom,uNom,xFin);

	//Rollout Class
	state_control_t stateBasedController;
	stateBasedController.setReference(timeRef,inputRef);
	ocs2::StateTriggeredRollout<STATE_DIM,INPUT_DIM> stateTriggeredRollout(systemModel,rolloutSettings);
	stateTriggeredRollout.setStateBasedController(&stateBasedController);

	// Operating points and PartitioningTimes
	std::vector<double> partitioningTimes;
	partitioningTimes.push_back(startTime);
	partitioningTimes.push_back(finalTime);

	// Initial Controller
	const input_state_matrix_t controllerGain = {25,10,0};
	input_vector_t controllerBias;

	input_state_matrix_array_t controllerGainArray;
	input_vector_array_t controllerBiasArray;
	scalar_array_t timeStampArray;

	const scalar_t controllerDeltaTime = 1e-3;		// Time step for controller time array
	const scalar_t eps = ocs2::OCS2NumericTraits<scalar_t>::weakEpsilon();
	std::vector<double> controlTimes = trajTimes;
	controlTimes.push_back(finalTime);

	for(int i = 0; i<controlTimes.size()-1; i++)
	{
		scalar_t timeSteps = (controlTimes[i+1]+eps-controlTimes[i])/controllerDeltaTime;
		scalar_t timeStamp = 0;
		state_vector_t refState;
		input_vector_t refInput;

		for(int j = 0; j<=timeSteps ; j++)
		{

			if(j == 0 && i<controlTimes.size()-2)
			{
				timeStamp = controlTimes[i] + eps;
			}
			else
			{
				timeStamp = controlTimes[i] + j*controllerDeltaTime;
			}

			reference.getState(timeStamp,refState);
			reference.getInput(timeStamp,refInput[0]);
			controllerBias = controllerGain*refState + refInput;

			timeStampArray.push_back(timeStamp);
			controllerGainArray.push_back(-controllerGain);
			controllerBiasArray.push_back(controllerBias);
		}
	}

	ocs2::LinearController<3,1> Control(timeStampArray,controllerBiasArray,controllerGainArray);
	ocs2::LinearController<3,1>* Controller = &Control;
	std::vector<controller_t*> controllerPtrArray = {&Control};

	systemOperatingTrajectories operatingTrajectories;
	//SLQ
	std::cout<<"Begin of SLQ"<<std::endl;
	ocs2::SLQ<STATE_DIM,INPUT_DIM> slqST(&stateTriggeredRollout, &systemDerivatives, &systemConstraints, &systemCost, &operatingTrajectories,slqSettings);
	slqST.run(startTime, x0, finalTime, partitioningTimes,controllerPtrArray);
	ocs2::SLQ<STATE_DIM, INPUT_DIM>::primal_solution_t solutionST = slqST.primalSolution(finalTime);
	std::cout<<"End of SLQ"<<std::endl;

	if (true){
		for(int i = 0; i<solutionST.stateTrajectory_.size();i++	)
		{
			int idx;
			idx = solutionST.stateTrajectory_[i][2];

			double uRef;
			state_vector_t xRef;
			reference.getState(idx,solutionST.timeTrajectory_[i],xRef);
			reference.getInput(solutionST.timeTrajectory_[i],uRef);

			std::cout<<i<<";"<<idx<<";";
			std::cout<<std::setprecision(25)<<solutionST.timeTrajectory_[i];
			std::cout<<std::setprecision(6)<<";"<<solutionST.stateTrajectory_[i][0]<<";"<< xRef[0]<<";";
			std::cout<<solutionST.stateTrajectory_[i][1]<<";"<< xRef[1]<<";";
			std::cout<<solutionST.inputTrajectory_[i][0]<<";"<<uRef<<std::endl;
		}
	}

}


