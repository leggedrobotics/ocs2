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

namespace ocs2{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
ModeSequence_ROS_Interface<SCALAR_T>::ModeSequence_ROS_Interface(
			const std::string& robotName /*= "robot"*/)
	: robotName_(robotName)
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
ModeSequence_ROS_Interface<SCALAR_T>::~ModeSequence_ROS_Interface() {

	shutdownNodes();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
void ModeSequence_ROS_Interface<SCALAR_T>::publishModeSequenceTemplate(
		const mode_sequence_template_t& modeSequenceTemplate) {

	RosMsgConversions<0, 0>::CreateModeSequenceTemplateMsg(modeSequenceTemplate,
			modeSequenceTemplateMsg_);

	mpcModeSequencePublisher_.publish(modeSequenceTemplateMsg_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
void ModeSequence_ROS_Interface<SCALAR_T>::shutdownNodes() {

	mpcModeSequencePublisher_.shutdown();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
void ModeSequence_ROS_Interface<SCALAR_T>::launchNodes(int argc, char* argv[]) {

	// reset counters and variables
	reset();

	// display
	ROS_INFO_STREAM("ModeSequence node is setting up ...");

	// setup ROS
	::ros::init(argc, argv, robotName_+"_mpc_mode_sequence");
	::ros::NodeHandle nodeHandler;

	mpcModeSequencePublisher_ = nodeHandler.advertise<ocs2_comm_interfaces::mode_sequence>(
			robotName_ + "_mpc_mode_sequence", 1, true);

	ros::spinOnce();

	// display
	ROS_INFO_STREAM(robotName_ + " mode sequence command node is ready.");
}

} // namespace ocs2


