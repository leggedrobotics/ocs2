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
TargetTrajectories_Keyboard_Interface<SCALAR_T>::TargetTrajectories_Keyboard_Interface(
		const std::string& robotName /*= "robot"*/,
		const size_t targetCommandSize /*= 0*/,
		const scalar_array_t& targetCommandLimits /*= scalar_array_t()*/)
	: BASE(robotName)
	, targetCommandSize_(targetCommandSize)
	, targetCommandLimits_(targetCommandLimits)
{
	if (targetCommandLimits.size()!=targetCommandSize)
		throw std::runtime_error("Target command limits are not set properly");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
size_t& TargetTrajectories_Keyboard_Interface<SCALAR_T>::targetCommandSize() {

	return targetCommandSize_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
void TargetTrajectories_Keyboard_Interface<SCALAR_T>::toCostDesiredTimeStateInput(
		const scalar_array_t& commadLineTarget,
		scalar_t& desiredTime,
		dynamic_vector_t& desiredState,
		dynamic_vector_t& desiredInput) {

	// time
	desiredTime = 0.0;
	// state
	desiredState = Eigen::Map<const dynamic_vector_t>(
			commadLineTarget.data(), targetCommandSize_);
	// input
	desiredInput = dynamic_vector_t::Zero(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
void TargetTrajectories_Keyboard_Interface<SCALAR_T>::getKeyboardCommand(
		const std::string& commadMsg /*= "Enter command, separated by spaces"*/) {

	while (ros::ok()) {

		// get command line
		std::cout << commadMsg << ": ";
		targetCommand_ = getCommandLine();

		// limits
		for (size_t i=0; i<targetCommandSize_; i++) {
			if (std::abs(targetCommand_[i]) > targetCommandLimits_[i])
				targetCommand_[i] = std::copysign(targetCommandLimits_[i], targetCommand_[i]);
		}  // end of i loop

		std::cout << "The following command is published: [";
		for (size_t i=0; i<targetCommandSize_; i++)
			std::cout << std::setprecision(4) << targetCommand_[i] << ", ";
		std::cout << "\b\b]" << std::endl << std::endl;

		// user defined modification of the command-line
		cost_desired_trajectories_t costDesiredTrajectories(1);
		toCostDesiredTimeStateInput(targetCommand_,
				costDesiredTrajectories.desiredTimeTrajectory()[0],
				costDesiredTrajectories.desiredStateTrajectory()[0],
				costDesiredTrajectories.desiredInputTrajectory()[0]);

		// publish cost desired trajectories
		BASE::publishTargetTrajectories(costDesiredTrajectories);

	} // end of while loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
typename TargetTrajectories_Keyboard_Interface<SCALAR_T>::scalar_array_t
	TargetTrajectories_Keyboard_Interface<SCALAR_T>::getCommandLine() {

	scalar_array_t targetCommand(0);

	std::string line;
	std::getline(std::cin, line);
	std::istringstream stream(line);
	scalar_t in;
	while (stream >> in)
		targetCommand.push_back(in);

	// if the size is greater than targetCommandSize_
	const size_t n = targetCommand.size();
	if (n > targetCommandSize_)
		targetCommand.erase(targetCommand.begin()+targetCommandSize_, targetCommand.end());
	else
		for (size_t i=n; i<targetCommandSize_; i++) {
			targetCommand.push_back(0.0);
		}  // end of i loop

	return targetCommand;
}

} // namespace ocs2

