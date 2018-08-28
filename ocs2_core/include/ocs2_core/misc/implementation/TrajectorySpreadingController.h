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
void TrajectorySpreadingController<STATE_DIM, INPUT_DIM>::adjustController(
		const scalar_array_t& eventTimes,
		controller_array_t& controllersStock) {

	initActivePartition_ = 0;
	for (; initActivePartition_<controllersStock.size(); initActivePartition_++)
		if (controllersStock[initActivePartition_].empty()==false)
			break;

	std::cout << "initActivePartition_:  " << initActivePartition_ << std::endl;

	// Finds the indices of the new event times
	findsIndicesEventTimes(eventTimes, controllersStock,
			eventsIndices_);

	std::cout << "indices of the new eventTimes: " << std::endl;
	for (auto& ind : eventsIndices_) {
		std::cout << "\t Index: " << "(" << ind.first << ", " << ind.second << ")";
		if (ind.first>=0 && ind.first<controllersStock.size()) {
			std::cout << "\t Size: " << controllersStock[ind.first].time_.size();
			std::cout << "\t Before: " << controllersStock[ind.first].time_[ind.second-1];
			std::cout << "\t Itself: " << controllersStock[ind.first].time_[ind.second];
			std::cout << "\t After:  " << controllersStock[ind.first].time_[ind.second+1];
		}
		std::cout << std::endl;
	}


	// Event times for which the controller is designed
	controllerEventsIndices_.clear();
	controllerEventsIndices_.resize(eventTimes.size(), index_t(-1, -1));
	size_t p = 0; // partitionStart
	for (size_t j=0; j<eventTimes.size(); j++) {

		index_t& ie = controllerEventsIndices_[j]; // (partition, index)

		// events before the controller start time
		if (eventsIndices_[j] == index_t(initActivePartition_,0)) {
			ie = index_t(initActivePartition_, 0);
			continue;
		}
		// events after the controller final time
		if (eventsIndices_[j] == index_t(-1,-1)) {
			break;
		}

		for (; p<controllersStock.size(); p++) {

			// skip if the controller is empty
			if (controllersStock[p].empty() == true)
				continue;

			// if not the first event, use the index of the previous event in order to be more efficient.
			// subjected to that they are in the same partition. Moreover we have assume that that no
			// event takes place at index zero.
			size_t beginInd = 0;
			if (j>0 && eventsIndices_[j-1].first==p)
				beginInd += eventsIndices_[j-1].second;

			// find the index in which the difference in between two consecutive time become zero
			size_t index = controllersStock[p].time_.size();
			for (size_t k=beginInd+1; k<controllersStock[p].time_.size()-1; k++) {
				if (std::abs(controllersStock[p].time_[k]-controllersStock[p].time_[k+1]) < OCS2NumericTraits<scalar_t>::week_epsilon()) {
					index = k;
					break;
				}
			} // end of k loop

			// if the lower bound found
			if (index != controllersStock[p].time_.size()) {
				ie.first = p;
				ie.second = index;
				break;
			}

		} // end of p loop

	} // end of j loop

	std::cout << "indices of the old eventTimes: " << std::endl;
	for (auto& ind : controllerEventsIndices_) {
		std::cout << "\t Index: " << "(" << ind.first << ", " << ind.second << ")";
		if (ind.first>=0 && ind.first<controllersStock.size()) {
			std::cout << "\t Size:   " << controllersStock[ind.first].time_.size();
			std::cout << "\t Before: " << controllersStock[ind.first].time_[ind.second-1];
			std::cout << "\t Itself: " << controllersStock[ind.first].time_[ind.second];
			std::cout << "\t After:  " << controllersStock[ind.first].time_[ind.second+1];
		}
		std::cout << std::endl;
	}

	// adjust controller
	for (size_t j=0; j<eventTimes.size(); j++) {
		adjustController(eventTimes[j], eventsIndices_[j], controllerEventsIndices_[j],
				controllersStock);
	} // end of j loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void TrajectorySpreadingController<STATE_DIM, INPUT_DIM>::findsIndicesEventTimes(
			const scalar_array_t& eventTimes,
			const controller_array_t& controllersStock,
			std::vector<index_t>& eventsIndices) const {

	// vector of (partition, index). -1 means end()
	eventsIndices.clear();
	eventsIndices.resize(eventTimes.size(), index_t(-1, -1));

	size_t p = 0; // partitionStart
	for (size_t j=0; j<eventTimes.size(); j++) {

		const scalar_t& te = eventTimes[j];
		index_t& ie = eventsIndices[j]; // (partition, index)

		for (; p<controllersStock.size(); p++) {

			// skip if the controller is empty
			if (controllersStock[p].empty() == true)
				continue;

			// if not the first event, use the index of the previous event in order to be more efficient.
			// subjected to that they are in the same partition
			typename scalar_array_t::const_iterator beginItr = controllersStock[p].time_.begin();
			if (j>0 && eventsIndices[j-1].first==p)
				beginItr += eventsIndices[j-1].second;

			auto lower = std::lower_bound(beginItr, controllersStock[p].time_.end(), te);

			// if the lower bound found
			if (lower != controllersStock[p].time_.end()) {
				ie.first = p;
				ie.second = lower - controllersStock[p].time_.begin();
				break;
			}

		} // end of p loop
	} // end of j loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
bool TrajectorySpreadingController<STATE_DIM, INPUT_DIM>::smallerEqualIndexFunc(
		const index_t& a,
		const index_t& b) const {

	if (a.first < b.first)
		return true;

	if (a.first > b.first)
		return false;

	if (a.first == b.first) {
		if (a.second <= b.second)
			return true;
		else
			return false;
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void TrajectorySpreadingController<STATE_DIM, INPUT_DIM>::adjustController(
		const scalar_t& eventTime,
		const index_t& eventTimeIndex,
		const index_t& controlerEventTimeIndex,
		controller_array_t& controllersStock) const {

	// events before the controller start time
	if (eventTimeIndex == index_t(initActivePartition_,0)) {
		return;
	}
	// events after the controller final time
	if (eventTimeIndex == index_t(-1,-1)) {
		return;
	}

	input_vector_t uffCorrected;
	input_state_matrix_t kCorrected;
	const size_t& p = eventTimeIndex.first;
	const size_t& ind = eventTimeIndex.second;
	const scalar_t alpha = (eventTime - controllersStock[p].time_[ind-1]) /
			(controllersStock[p].time_[ind] - controllersStock[p].time_[ind-1]);

	uffCorrected = (1-alpha) * controllersStock[p].uff_[ind-1] + alpha * controllersStock[p].uff_[ind];
	kCorrected   = (1-alpha) * controllersStock[p].k_[ind-1] + alpha * controllersStock[p].k_[ind];


	index_t startIndex;
	index_t finalIndex;
	input_vector_t uffSpread;
	input_state_matrix_t kSpread;
	if (smallerEqualIndexFunc(eventTimeIndex, controlerEventTimeIndex) == true) {
		startIndex = eventTimeIndex;
		finalIndex = controlerEventTimeIndex;

		// it is definitely at the same partition since it is the controller event times
		uffSpread = controllersStock[controlerEventTimeIndex.first].uff_[controlerEventTimeIndex.second+2];  // it should be +1
		kSpread   = controllersStock[controlerEventTimeIndex.first].k_[controlerEventTimeIndex.second+2];    // it should be +1

		controllersStock[eventTimeIndex.first].time_[eventTimeIndex.second] = eventTime;

	} else {
		// find the index before eventTimeIndex
		// if it is not the last index in the partition
		index_t eventTimeIndexPrev = std::make_pair(eventTimeIndex.first, eventTimeIndex.second-1);
		if (eventTimeIndex.second == 0) {
			if (eventTimeIndex.first > initActivePartition_)
				eventTimeIndexPrev = std::make_pair(eventTimeIndex.first-1, controllersStock[eventTimeIndex.first-1].time_.size()-1);
			else
				eventTimeIndexPrev = std::make_pair(initActivePartition_, 0);
		}


		eventTimeIndexPrev = eventTimeIndex;

		startIndex = controlerEventTimeIndex;
		finalIndex = eventTimeIndexPrev;

		// it is definitely at the same partition since it is the controller event times
		uffSpread = controllersStock[controlerEventTimeIndex.first].uff_[controlerEventTimeIndex.second];
		kSpread   = controllersStock[controlerEventTimeIndex.first].k_[controlerEventTimeIndex.second];

		controllersStock[eventTimeIndexPrev.first].time_[eventTimeIndexPrev.second] = eventTime;
	}

	// set inputs
	for (size_t i=startIndex.first; i<=finalIndex.first; i++)
		for (size_t k=startIndex.second; k<=finalIndex.second; k++) {
			controllersStock[i].uff_[k]  = uffSpread;
			controllersStock[i].k_[k]    = kSpread;
		}


}

}  // ocs2 namespace

