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

#include <ocs2_robotic_examples/examples/double_integrator/DoubleIntegratorInterface.h>
#include <ocs2_robotic_examples/examples/double_integrator/ros_comm/MRT_ROS_Double_Integrator.h>
#include <ocs2_robotic_examples/examples/double_integrator/definitions.h>
#include <ocs2_robotic_examples/examples/double_integrator/ros_comm/MRT_ROS_Dummy_Double_Integrator.h>

using namespace ocs2;
using namespace double_integrator;

int main(int argc, char **argv)
{
	// task file
	if (argc <= 1) throw std::runtime_error("No task file specified. Aborting.");
	std::string taskFileFolderName = std::string(argv[1]);

	// double_integratorInterface
	DoubleIntegratorInterface double_integratorInterface(taskFileFolderName);

	typedef ocs2::MRT_ROS_Interface<double_integrator::STATE_DIM_, double_integrator::INPUT_DIM_> mrt_t;
	typedef typename mrt_t::Ptr	mrt_ptr_t;
	typedef typename mrt_t::scalar_t scalar_t;
	typedef typename mrt_t::system_observation_t system_observation_t;
	typedef ocs2::NullLogicRules<double_integrator::STATE_DIM_, double_integrator::INPUT_DIM_> logic_rules_t;
	mrt_ptr_t mrtPtr(new mrt_t(logic_rules_t(), !double_integratorInterface.mpcSettings().useFeedbackPolicy_, "double_integrator"));

	// Dummy double_integrator
	MRT_ROS_Dummy_Linear_System dummyDoubleIntegrator(
			mrtPtr,
			double_integratorInterface.mpcSettings().mrtDesiredFrequency_,
			double_integratorInterface.mpcSettings().mpcDesiredFrequency_);
	dummyDoubleIntegrator.launchNodes(argc, argv);
	// Initialize dummy
	MRT_ROS_Dummy_Linear_System::system_observation_t initObservation;
	double_integratorInterface.getInitialState(initObservation.state());
	dummyDoubleIntegrator.init(initObservation);

	// run dummy
	dummyDoubleIntegrator.run();

	// Successful exit
	return 0;
}


