/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/


#include "ocs2_double_integrator_example/DoubleIntegratorInterface.h"
#include "ocs2_comm_interfaces/ocs2_interfaces/MPC_Interface.h"
#include "ocs2_core/logic/rules/NullLogicRules.h"

using namespace ocs2;
using namespace double_integrator;
using dim_t = ocs2::Dimensions<double_integrator_dims::STATE_DIM_, double_integrator_dims::INPUT_DIM_>;
typedef MPC_Interface<dim_t::STATE_DIM_, dim_t::INPUT_DIM_> mpc_t;


int main(int argc, char **argv)
{
	// task file
	if (argc <= 1) throw std::runtime_error("No task file specified. Aborting.");
	std::string taskFileFolderName = std::string(argv[1]);

	NullLogicRules nullLogicRules;

	DoubleIntegratorInterface doubleIntegratorInterface(taskFileFolderName);
	mpc_t mpcInterface(
			*doubleIntegratorInterface.getMPCPtr(),
			nullLogicRules,
			true);

	double time = 0;

	//initialize observation:
	mpc_t::system_observation_t observation;
	doubleIntegratorInterface.getInitialState(observation.state());
  observation.time() = time;

	mpcInterface.setCurrentObservation(observation);


	//initialize reference:
	mpc_t::cost_desired_trajectories_t costDesiredTrajectories;
	costDesiredTrajectories.desiredTimeTrajectory().push_back(time);
  costDesiredTrajectories.desiredTimeTrajectory().push_back(time+1);
  costDesiredTrajectories.desiredTimeTrajectory().push_back(time+5);
  mpc_t::state_vector_t goalState;
	goalState << 1,0;
	costDesiredTrajectories.desiredStateTrajectory().push_back(observation.state());
	costDesiredTrajectories.desiredStateTrajectory().push_back(goalState);
  costDesiredTrajectories.desiredStateTrajectory().push_back(goalState);
  mpc_t::input_vector_t desiredInput;
	costDesiredTrajectories.desiredInputTrajectory().push_back(desiredInput);
	costDesiredTrajectories.desiredInputTrajectory().push_back(desiredInput);
  costDesiredTrajectories.desiredInputTrajectory().push_back(desiredInput);
  mpcInterface.setTargetTrajectories(costDesiredTrajectories);

  double f_control = 10;
  //double f_control = doubleIntegratorInterface.mpcSettings().mpcDesiredFrequency_;
  double T = 3;

	//run MPC for N iterations
	int N = int(f_control * T);
	for (int i=0; i<N; i++){
		//run MPC
		mpcInterface.advanceMpc();
    time += 1.0/f_control;

		if (mpcInterface.policyReceived()){
			mpc_t::state_vector_t optimalState;
			mpc_t::input_vector_t optimalInput;
			size_t subsystem;
			mpcInterface.evaluateFeedforwardPolicy(time, optimalState, optimalInput, subsystem);

			std::cout << std::endl << "time:" << time
					<< "  state:" << optimalState.transpose()
					<< "  input:" << optimalInput.transpose()
					<< std::endl << std::endl;

			//use optimal state for the next observation:
			observation.state() = optimalState;
      observation.time() = time;
			mpcInterface.setCurrentObservation(observation);
		}

	}


	// Successful exit
	return 0;
}

/* EOF */
