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
size_t Solver_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::findActivePartitionIndex(
		const scalar_array_t& partitioningTimes,
		const scalar_t& time,
		bool ceilingFunction /*= true*/) {

	int activeSubsystemIndex;
	if (ceilingFunction) {
		activeSubsystemIndex = findActiveIntervalIndex(partitioningTimes, time, 0);
	} else {
		activeSubsystemIndex = findActiveIntervalIndex(partitioningTimes, time, 0,
				-OCS2NumericTraits<scalar_t>::weakEpsilon());
	}

	if (activeSubsystemIndex < 0) {
		std::string mesg = "Given time is less than the start time (i.e. givenTime < partitioningTimes.front()): "
				+ std::to_string(time) + " < " + std::to_string(partitioningTimes.front());
		throw std::runtime_error(mesg);
	}

	if (activeSubsystemIndex == partitioningTimes.size()-1) {
		std::string mesg = "Given time is greater than the final time (i.e. partitioningTimes.back() < givenTime): "
				+ std::to_string(partitioningTimes.back()) + " < " + std::to_string(time);
		throw std::runtime_error(mesg);
	}

	return (size_t)activeSubsystemIndex;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void Solver_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::printString(const std::string& text) {

	std::lock_guard<std::mutex> outputDisplayGuard(outputDisplayGuardMutex_);
	std::cerr << text << std::endl;
}

}  // ocs2 namespace
