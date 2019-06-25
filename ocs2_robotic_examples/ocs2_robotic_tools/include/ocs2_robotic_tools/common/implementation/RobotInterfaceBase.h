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

namespace ocs2
{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void RobotInterfaceBase<STATE_DIM, INPUT_DIM>::getInitialState(
		state_vector_t& initialState) const {

	initialState = initialState_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
MPC_Settings& RobotInterfaceBase<STATE_DIM, INPUT_DIM>::mpcSettings() {

	return mpcSettings_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void RobotInterfaceBase<STATE_DIM, INPUT_DIM>::definePartitioningTimes(
		const std::string& taskFile,
		scalar_t& timeHorizon,
		size_t& numPartitions,
		scalar_array_t& partitioningTimes,
		bool verbose /*= false*/) {

	// load from task file
	loadMpcTimeHorizon(taskFile, timeHorizon, numPartitions);

	if (numPartitions==0) {
		throw std::runtime_error("mpcTimeHorizon field is not defined.");
	}

	partitioningTimes.resize(numPartitions+1);
	partitioningTimes[0] = 0.0;
	for (size_t i=0; i<numPartitions; i++)
		partitioningTimes[i+1] = partitioningTimes[i] + timeHorizon/numPartitions;
	partitioningTimes[numPartitions] = timeHorizon;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void RobotInterfaceBase<STATE_DIM, INPUT_DIM>::loadMpcTimeHorizon(
		const std::string& taskFile,
		scalar_t& timeHorizon,
		size_t& numPartitions,
		bool verbose /*= false*/) const {

	boost::property_tree::ptree pt;
	boost::property_tree::read_info(taskFile, pt);

	timeHorizon   = pt.get<scalar_t>("mpcTimeHorizon.timehorizon");
	numPartitions = pt.get<size_t>("mpcTimeHorizon.numPartitions");

	if (verbose == true) {
		std::cerr<<"Time Horizon Settings: " << std::endl;
		std::cerr<<"=====================================" << std::endl;
		std::cerr<<"Time Horizon .................. " << timeHorizon << std::endl;
		std::cerr<<"Number of Partitions .......... " << numPartitions << std::endl << std::endl;
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void RobotInterfaceBase<STATE_DIM, INPUT_DIM>::loadInitialState(
		const std::string& taskFile,
		state_vector_t& initialState) const {

	boost::property_tree::ptree pt;
	boost::property_tree::read_info(taskFile, pt);

	loadEigenMatrix(taskFile, "initialState", initialState);
}

}  // namespace ocs2
