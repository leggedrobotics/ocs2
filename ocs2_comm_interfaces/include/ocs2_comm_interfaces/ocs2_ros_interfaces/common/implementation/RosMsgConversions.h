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
template <size_t STATE_DIM, size_t INPUT_DIM>
template <class ContainerAllocator>
void RosMsgConversions<STATE_DIM, INPUT_DIM>::CreateObservationMsg(
		const system_observation_t& observation,
		ocs2_comm_interfaces::mpc_observation_<ContainerAllocator>& observationMsg) {

	observationMsg.time = observation.time();

	observationMsg.state.value.resize(STATE_DIM);
	for (size_t i=0; i<STATE_DIM; i++)
		observationMsg.state.value[i] = observation.state(i);

	observationMsg.input.value.resize(INPUT_DIM);
	for (size_t i=0; i<INPUT_DIM; i++)
		observationMsg.input.value[i] = observation.input(i);

	observationMsg.subsystem = observation.subsystem();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
template <class ContainerAllocator>
void RosMsgConversions<STATE_DIM, INPUT_DIM>::ReadObservationMsg(
		const ocs2_comm_interfaces::mpc_observation_<ContainerAllocator>& observationMsg,
		system_observation_t& observation) {

	observation.time() = observationMsg.time;

	observation.state() = Eigen::Map<const Eigen::Matrix<float,STATE_DIM,1>> (
			observationMsg.state.value.data(), STATE_DIM).template cast<scalar_t>();

	observation.input() = Eigen::Map<const Eigen::Matrix<float,INPUT_DIM,1>> (
			observationMsg.input.value.data(), INPUT_DIM).template cast<scalar_t>();

	observation.subsystem() = observationMsg.subsystem;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
template <class ContainerAllocator>
void RosMsgConversions<STATE_DIM, INPUT_DIM>::CreateModeSequenceMsg(
		const scalar_array_t& eventTimes,
		const size_array_t& subsystemsSequence,
		ocs2_comm_interfaces::mode_sequence_<ContainerAllocator>& modeSequenceMsg) {

	// event time sequence
	modeSequenceMsg.eventTimes.clear();
	modeSequenceMsg.eventTimes.reserve(eventTimes.size());
	for (const scalar_t& ti: eventTimes)
		modeSequenceMsg.eventTimes.push_back(ti);

	// subsystem sequence
	modeSequenceMsg.subsystems.clear();
	modeSequenceMsg.subsystems.reserve(subsystemsSequence.size());
	for (const size_t& si: subsystemsSequence)
		modeSequenceMsg.subsystems.push_back(si);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
template <class ContainerAllocator>
void RosMsgConversions<STATE_DIM, INPUT_DIM>::ReadModeSequenceMsg(
		const ocs2_comm_interfaces::mode_sequence_<ContainerAllocator>& modeSequenceMsg,
		scalar_array_t& eventTimes,
		size_array_t& subsystemsSequence) {

	const size_t numSubsystems = modeSequenceMsg.subsystems.size();
	if (modeSequenceMsg.eventTimes.size() != numSubsystems-1)
		throw std::runtime_error("The received message has incompatible "
				"array sizes for the eventTimes and subsystemsSequence.");

	// event time sequence
	eventTimes.clear();
	eventTimes.reserve(numSubsystems-1);
	for (const scalar_t& ti: modeSequenceMsg.eventTimes)
		eventTimes.push_back(ti);

	// subsystem sequence
	subsystemsSequence.clear();
	subsystemsSequence.reserve(numSubsystems);
	for (const size_t& si: modeSequenceMsg.subsystems)
		subsystemsSequence.push_back(si);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
template <class ContainerAllocator>
void RosMsgConversions<STATE_DIM, INPUT_DIM>::CreateModeSequenceTemplateMsg(
		const mode_sequence_template_t& modeSequenceTemplate,
		ocs2_comm_interfaces::mode_sequence_<ContainerAllocator>& modeSequenceMsg) {

	// event time sequence
	modeSequenceMsg.eventTimes.clear();
	modeSequenceMsg.eventTimes.reserve(modeSequenceTemplate.templateSwitchingTimes_.size());
	for (const scalar_t& ti: modeSequenceTemplate.templateSwitchingTimes_)
		modeSequenceMsg.eventTimes.push_back(ti);

	// subsystem sequence
	modeSequenceMsg.subsystems.clear();
	modeSequenceMsg.subsystems.reserve(modeSequenceTemplate.templateSubsystemsSequence_.size());
	for (const size_t& si: modeSequenceTemplate.templateSubsystemsSequence_)
		modeSequenceMsg.subsystems.push_back(si);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
template <class ContainerAllocator>
void RosMsgConversions<STATE_DIM, INPUT_DIM>::ReadModeSequenceTemplateMsg(
		const ocs2_comm_interfaces::mode_sequence_<ContainerAllocator>& modeSequenceMsg,
		mode_sequence_template_t& modeSequenceTemplate) {

	const size_t numSubsystems = modeSequenceMsg.subsystems.size();
	if (modeSequenceMsg.eventTimes.size() != numSubsystems+1)
		throw std::runtime_error("The received message has incompatible "
				"array sizes for the switchingTimes and subsystemsSequence.");

	// switching time sequence
	modeSequenceTemplate.templateSwitchingTimes_.clear();
	modeSequenceTemplate.templateSwitchingTimes_.reserve(numSubsystems+1);
	for (const scalar_t& ti: modeSequenceMsg.eventTimes)
		modeSequenceTemplate.templateSwitchingTimes_.push_back(ti);

	// subsystem sequence
	modeSequenceTemplate.templateSubsystemsSequence_.clear();
	modeSequenceTemplate.templateSubsystemsSequence_.reserve(numSubsystems);
	for (const size_t& si: modeSequenceMsg.subsystems)
		modeSequenceTemplate.templateSubsystemsSequence_.push_back(si);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
template <class ContainerAllocator>
void RosMsgConversions<STATE_DIM, INPUT_DIM>::CreateTargetTrajectoriesMsg(
		const cost_desired_trajectories_t& costDesiredTrajectories,
		ocs2_comm_interfaces::mpc_target_trajectories_<ContainerAllocator>& targetTrajectoriesMsg) {

	const scalar_array_t& desiredTimeTrajectory = costDesiredTrajectories.desiredTimeTrajectory();
	const dynamic_vector_array_t& desiredStateTrajectory = costDesiredTrajectories.desiredStateTrajectory();
	const dynamic_vector_array_t& desiredInputTrajectory = costDesiredTrajectories.desiredInputTrajectory();

	// time and state
	size_t N = desiredStateTrajectory.size();
	targetTrajectoriesMsg.timeTrajectory.resize(N);
	targetTrajectoriesMsg.stateTrajectory.resize(N);
	for (size_t i=0; i<N; i++) {
		targetTrajectoriesMsg.timeTrajectory[i] = desiredTimeTrajectory[i];

		targetTrajectoriesMsg.stateTrajectory[i].value = std::vector<float>(desiredStateTrajectory[i].data(),
				desiredStateTrajectory[i].data()+desiredStateTrajectory[i].size());
	}  // end of i loop

	// input
	N = desiredInputTrajectory.size();
	targetTrajectoriesMsg.inputTrajectory.resize(N);
	for (size_t i=0; i<N; i++) {
		targetTrajectoriesMsg.inputTrajectory[i].value = std::vector<float>(desiredInputTrajectory[i].data(),
				desiredInputTrajectory[i].data()+desiredInputTrajectory[i].size());
	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
template <class ContainerAllocator>
void RosMsgConversions<STATE_DIM, INPUT_DIM>::ReadTargetTrajectoriesMsg(
			const ocs2_comm_interfaces::mpc_target_trajectories_<ContainerAllocator>& targetTrajectoriesMsg,
			cost_desired_trajectories_t& costDesiredTrajectories) {

	scalar_array_t& desiredTimeTrajectory = costDesiredTrajectories.desiredTimeTrajectory();
	dynamic_vector_array_t& desiredStateTrajectory = costDesiredTrajectories.desiredStateTrajectory();
	dynamic_vector_array_t& desiredInputTrajectory = costDesiredTrajectories.desiredInputTrajectory();

	size_t N = targetTrajectoriesMsg.stateTrajectory.size();
	if (N==0)
		throw std::runtime_error("An empty target trajectories message is received.");

	// state and time
	desiredTimeTrajectory.resize(N);
	desiredStateTrajectory.resize(N);
	for (size_t i=0; i<N; i++) {
		desiredTimeTrajectory[i] = targetTrajectoriesMsg.timeTrajectory[i];

		desiredStateTrajectory[i] = Eigen::Map<const Eigen::VectorXf>(targetTrajectoriesMsg.stateTrajectory[i].value.data(),
				targetTrajectoriesMsg.stateTrajectory[i].value.size()).template cast<scalar_t>();
	}  // end of i loop

	// input
	N = targetTrajectoriesMsg.inputTrajectory.size();
	desiredInputTrajectory.resize(N);
	for (size_t i=0; i<N; i++) {
		desiredInputTrajectory[i] = Eigen::Map<const Eigen::VectorXf>(targetTrajectoriesMsg.inputTrajectory[i].value.data(),
				targetTrajectoriesMsg.inputTrajectory[i].value.size()).template cast<scalar_t>();
	}  // end of i loop
}

} // namespace ocs2
