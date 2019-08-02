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

#include "ocs2_ballbot_example/BallbotInterface.h"
#include "ocs2_ballbot_example/definitions.h"
#include "ocs2_ballbot_example/ros_comm/MRT_ROS_Ballbot.h"
#include "ocs2_ballbot_example/ros_comm/MRT_ROS_Dummy_Ballbot.h"

using namespace ocs2;
using namespace ballbot;

int main(int argc, char **argv)
{
	// task file
	if (argc <= 1) { throw std::runtime_error("No task file specified. Aborting.");
	}
	std::string taskFileFolderName = std::string(argv[1]);

	// ballbotInterface
	BallbotInterface ballbotInterface(taskFileFolderName);

	using mrt_t = MRT_ROS_Ballbot;
	using mrt_ptr_t = mrt_t::Ptr;
	using scalar_t = mrt_t::scalar_t;
	using system_observation_t = mrt_t::system_observation_t;

	mrt_ptr_t mrtPtr(new mrt_t("ballbot"));

	// Dummy ballbot
	MRT_ROS_Dummy_Ballbot dummyBallbot(
			mrtPtr,
			ballbotInterface.mpcSettings().mrtDesiredFrequency_,
			ballbotInterface.mpcSettings().mpcDesiredFrequency_);

	dummyBallbot.launchNodes(argc, argv);

	// initial state
	MRT_ROS_Dummy_Ballbot::system_observation_t initObservation;
	ballbotInterface.getInitialState(initObservation.state());

	// initial command
	MRT_ROS_Dummy_Ballbot::cost_desired_trajectories_t initCostDesiredTrajectories;
	initCostDesiredTrajectories.desiredTimeTrajectory().push_back(initObservation.time());
	initCostDesiredTrajectories.desiredStateTrajectory().push_back(initObservation.state());
	initCostDesiredTrajectories.desiredInputTrajectory().push_back(initObservation.input());

	// run dummy
	dummyBallbot.run(initObservation, initCostDesiredTrajectories);

	// Successful exit
	return 0;
}
