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

#include "ocs2_quadrotor_example/QuadrotorInterface.h"
#include "ocs2_quadrotor_example/definitions.h"
#include "ocs2_quadrotor_example/ros_comm/MRT_ROS_Quadrotor.h"
#include "ocs2_quadrotor_example/ros_comm/MRT_ROS_Dummy_Quadrotor.h"

using namespace ocs2;
using namespace quadrotor;

int main(int argc, char **argv)
{
	// task file
	if (argc <= 1) throw std::runtime_error("No task file specified. Aborting.");
	std::string taskFileFolderName = std::string(argv[1]);

	// quadrotorInterface
	QuadrotorInterface quadrotorInterface(taskFileFolderName);

	typedef MRT_ROS_Quadrotor mrt_t;
	typedef mrt_t::BASE::Ptr mrt_ptr_t;
	typedef mrt_t::scalar_t scalar_t;
	typedef mrt_t::system_observation_t system_observation_t;

	mrt_ptr_t mrtPtr(new mrt_t(
			!quadrotorInterface.mpcSettings().useFeedbackPolicy_,
			"quadrotor"));

	// Dummy quadrotor
	MRT_ROS_Dummy_Quadrotor dummyQuadrotor(
			mrtPtr,
			quadrotorInterface.mpcSettings().mrtDesiredFrequency_,
			quadrotorInterface.mpcSettings().mpcDesiredFrequency_);

	dummyQuadrotor.launchNodes(argc, argv);

	// Initialize dummy
	MRT_ROS_Dummy_Quadrotor::system_observation_t initObservation;
	quadrotorInterface.getInitialState(initObservation.state());
	dummyQuadrotor.init(initObservation);

	// run dummy
	dummyQuadrotor.run();

	// Successful exit
	return 0;
}


