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

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
MRT_ROS_Dummy_Loop<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::MRT_ROS_Dummy_Loop(
		const mrt_ptr_t& mrtPtr,
		const scalar_t& mrtDesiredFrequency /*= 100*/,
		const scalar_t& mpcDesiredFrequency /*= -1*/)

	: mrtPtr_(mrtPtr)
	, mrtDesiredFrequency_(mrtDesiredFrequency)
	, mpcDesiredFrequency_(mpcDesiredFrequency)
	, realtimeLoop_(mpcDesiredFrequency<=0) // true if mpcDesiredFrequency is not set or it is negative
	, initialized_(false)
{
	if (mrtDesiredFrequency_<0)
		throw std::runtime_error("MRT loop frequency should be a positive number.");

	if (mpcDesiredFrequency_>0)
		ROS_WARN_STREAM("MPC loop is not realtime! "
				"For realtime setting, set mpcDesiredFrequency to any negative number.");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MRT_ROS_Dummy_Loop<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::launchNodes(int argc, char* argv[]) {

	mrtPtr_->launchNodes(argc, argv);

	launchVisualizerNode(argc, argv);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MRT_ROS_Dummy_Loop<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::init(const system_observation_t& initObservation) {

	initialized_ = true;
	initObservation_ = initObservation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MRT_ROS_Dummy_Loop<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::run() {

	if (initialized_==false)
		throw std::runtime_error("The init() method should be called at least once before.");

	::ros::Rate rosRate(mrtDesiredFrequency_); // in Hz

	// set the frequency ratio between MRT loop and MPC loop in the case of non realtime test
	size_t frequencyRatio = 1;
	if (realtimeLoop_==false)
		frequencyRatio = mrtDesiredFrequency_/mpcDesiredFrequency_;

	size_t loopCounter = 0;
	scalar_t time = initObservation_.time();

	// reset MPC node
	mrtPtr_->resetMpcNode();

	// wait for the initial MPC plan
	ROS_INFO_STREAM("Waiting for the initial policy ...");
	while (::ros::ok()) {
    mrtPtr_->spinMRT();
		// for initial plan
		initObservation_.time() = time;
		mrtPtr_->publishObservation(initObservation_);
		if (mrtPtr_->initialPolicyReceived()==true)
			break;
		else
			::ros::Duration(1.0/mrtDesiredFrequency_).sleep();
	}
	ROS_INFO_STREAM("Initial policy has been received.");


	while(::ros::ok()) {

		// this should be called before updatePolicy()
    mrtPtr_->spinMRT();

		// Checks for new policy and updates the policy
		bool policyUpdated = false;
		if (realtimeLoop_ == true) {
			policyUpdated = mrtPtr_->updatePolicy();

		} else if (loopCounter%frequencyRatio==0) {
			while(::ros::ok()) {
				policyUpdated = mrtPtr_->updatePolicy();
				if (policyUpdated==true)
					break;
				else
          mrtPtr_->spinMRT();
			}
			std::cout << "### Message received at " << time << std::endl;
		}

		// time and loop counter increment
		loopCounter++;
		time += (1.0/mrtDesiredFrequency_);

		std::cout << "### Message received at " << time << std::endl;

		// fake simulation of the dynamics
		observation_.time() = time;
		mrtPtr_->evaluateFeedforwardPolicy(observation_.time(),
				observation_.state(), observation_.input(), observation_.subsystem());

		// user-defined modifications before publishing
		modifyObservation(observation_);

		// publish observation
		if(realtimeLoop_ == true) {
			mrtPtr_->publishObservation(observation_);

		} else if (loopCounter%frequencyRatio==0) {
			mrtPtr_->publishObservation(observation_);
			std::cout << "### Observation is published at " << time << std::endl;
		}

		// Visualization
		publishVisualizer(observation_, mrtPtr_->mpcCostDesiredTrajectories());

		rosRate.sleep();

	}  // end of while loop

}

} // namespace ocs2
