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
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void TrajectorySpreadingControllerAdjustment<STATE_DIM, INPUT_DIM>::findEventTimesIndices(const scalar_array_t& eventTimes,
                                                                                          const linear_controller_array_t& controllersStock,
                                                                                          std::vector<index_t>& eventsIndices) const {
  // vector of (partition, index).
  eventsIndices.clear();
  eventsIndices.resize(eventTimes.size(), index_t(undefined_, undefined_));

  size_t p = 0;  // first partition
  for (size_t j = 0; j < eventTimes.size(); j++) {
    const scalar_t& te = eventTimes[j];
    index_t& ie = eventsIndices[j];  // (partition, index)

    for (; p < controllersStock.size(); p++) {
      // skip if the controller is empty
      if (controllersStock[p].empty()) {
        continue;
      }

      // if it is not the first event, use the index of the previous event for efficiency.
      // subjected to that they are in the same partition
      typename scalar_array_t::const_iterator beginItr = controllersStock[p].timeStamp_.begin();
      if (j > 0 && eventsIndices[j - 1].first == p) {
        beginItr += eventsIndices[j - 1].second;
      }

      auto lower = std::lower_bound(beginItr, controllersStock[p].timeStamp_.end(), te);

      // if the lower bound found
      if (lower != controllersStock[p].timeStamp_.end()) {
        ie.first = p;
        ie.second = lower - controllersStock[p].timeStamp_.begin();
        break;  // breaks p loop
      }

    }  // end of p loop
  }    // end of j loop

  // no event at the final time of controller
  for (index_t& ind : eventsIndices) {
    if (ind == index_t(finalActivePartitionIndex_, controllersStock[finalActivePartitionIndex_].size() - 1)) {
      ind = index_t(undefined_, undefined_);
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
bool TrajectorySpreadingControllerAdjustment<STATE_DIM, INPUT_DIM>::isSmallerEqual(const index_t& a, const index_t& b) const {
  if (a.first < b.first) {
    return true;
  }

  if (a.first > b.first) {
    return false;
  }

  if (a.first == b.first) {
    if (a.second <= b.second) {
      return true;
    } else {
      return false;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void TrajectorySpreadingControllerAdjustment<STATE_DIM, INPUT_DIM>::adjustController(const scalar_array_t& eventTimes,
                                                                                     const scalar_array_t& controllerEventTimes,
                                                                                     linear_controller_array_t& controllersStock) {
  const size_t numEvents = eventTimes.size();
  if (controllerEventTimes.size() != numEvents) {
    throw std::runtime_error("Number of events in controller is different from number of eventTimes.");
  }

  initActivePartitionIndex_ = 0;
  for (; initActivePartitionIndex_ < controllersStock.size(); initActivePartitionIndex_++) {
    if (!controllersStock[initActivePartitionIndex_].empty()) {
      break;
    }
  }

  finalActivePartitionIndex_ = controllersStock.size() - 1;
  for (; finalActivePartitionIndex_ >= 0; finalActivePartitionIndex_--) {
    if (!controllersStock[finalActivePartitionIndex_].empty()) {
      break;
    }
  }

  // finds the indices of the new event times
  std::vector<index_t> eventsIndices;
  findEventTimesIndices(eventTimes, controllersStock, eventsIndices);

  // finds the indices of the controller event times
  std::vector<index_t> controllerEventsIndices;
  findEventTimesIndices(controllerEventTimes, controllersStock, controllerEventsIndices);

  // adjust controller. Here we assume the event times at same indices are associated.
  for (size_t j = 0; j < numEvents; j++) {
    spreadController(eventTimes[j],               // new event times
                     eventsIndices[j],            // new event times indices
                     controllerEventsIndices[j],  // old event times indices
                     controllersStock);
  }  // end of j loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
typename TrajectorySpreadingControllerAdjustment<STATE_DIM, INPUT_DIM>::index_t
TrajectorySpreadingControllerAdjustment<STATE_DIM, INPUT_DIM>::findPreviousIndex(index_t index,
                                                                                 const linear_controller_array_t& controllersStock) const {
  index_t prevIndex;
  if (index.second > 0) {
    prevIndex.first = index.first;
    prevIndex.second = index.second - 1;
  } else if (index.first > initActivePartitionIndex_) {
    prevIndex.first = index.first - 1;
    prevIndex.second = controllersStock[prevIndex.first].timeStamp_.size() - 1;
  } else {
    prevIndex = index;
  }

  return prevIndex;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void TrajectorySpreadingControllerAdjustment<STATE_DIM, INPUT_DIM>::spreadController(scalar_t eventTime, index_t eventTimeIndex,
                                                                                     index_t controlerEventTimeIndex,
                                                                                     linear_controller_array_t& controllersStock) const {
  // events before the controller start time
  if (eventTimeIndex == index_t(initActivePartitionIndex_, 0) || controlerEventTimeIndex == index_t(initActivePartitionIndex_, 0)) {
    return;
  }
  // events after the controller final time
  if (eventTimeIndex == index_t(undefined_, undefined_) || controlerEventTimeIndex == index_t(undefined_, undefined_)) {
    return;
  }

  // find the index before eventTimeIndex
  index_t eventTimePrevIndex = findPreviousIndex(eventTimeIndex, controllersStock);
  // find the index before controlerEventTimeIndex
  index_t controlerEventTimePrevIndex = findPreviousIndex(controlerEventTimeIndex, controllersStock);

  // controller at the new event time
  const scalar_t alpha = (eventTime - controllersStock[eventTimePrevIndex.first].timeStamp_[eventTimePrevIndex.second]) /
                         (controllersStock[eventTimeIndex.first].timeStamp_[eventTimeIndex.second] -
                          controllersStock[eventTimePrevIndex.first].timeStamp_[eventTimePrevIndex.second]);
  // feedforward part
  input_vector_t uffCorrected;
  uffCorrected = (1 - alpha) * controllersStock[eventTimePrevIndex.first].biasArray_[eventTimePrevIndex.second] +
                 alpha * controllersStock[eventTimeIndex.first].biasArray_[eventTimeIndex.second];
  // feedback part
  input_state_matrix_t kCorrected;
  kCorrected = (1 - alpha) * controllersStock[eventTimePrevIndex.first].gainArray_[eventTimePrevIndex.second] +
               alpha * controllersStock[eventTimeIndex.first].gainArray_[eventTimeIndex.second];

  index_t startIndex;
  index_t finalIndex;
  input_vector_t uffSpread;
  input_state_matrix_t kSpread;
  if (isSmallerEqual(eventTimeIndex, controlerEventTimeIndex)) {  // if the event time moved to an earlier time
    startIndex = eventTimeIndex;
    finalIndex = controlerEventTimeIndex;

    // it is definitely at the same partition since it is the controller event times
    uffSpread = controllersStock[controlerEventTimeIndex.first].biasArray_[controlerEventTimeIndex.second + 1];  // it should be +1
    kSpread = controllersStock[controlerEventTimeIndex.first].gainArray_[controlerEventTimeIndex.second + 1];    // it should be +1

    controllersStock[eventTimeIndex.first].timeStamp_[eventTimeIndex.second] = eventTime;

  } else {  // if the event time moved to a later time
    startIndex = controlerEventTimeIndex;
    finalIndex = eventTimePrevIndex;

    // it is definitely at the same partition since it is the controller event times
    uffSpread = controllersStock[controlerEventTimeIndex.first].biasArray_[controlerEventTimeIndex.second];
    kSpread = controllersStock[controlerEventTimeIndex.first].gainArray_[controlerEventTimeIndex.second];

    controllersStock[eventTimePrevIndex.first].timeStamp_[eventTimePrevIndex.second] = eventTime;
  }

  // set inputs
  for (size_t i = startIndex.first; i <= finalIndex.first; i++) {
    const size_t startItr = (i == startIndex.first) ? startIndex.second : 0;
    const size_t finalItr = (i == finalIndex.first) ? finalIndex.second : controllersStock[i].size() - 1;

    for (size_t k = startItr; k <= finalItr; k++) {
      controllersStock[i].biasArray_[k] = uffSpread;
      controllersStock[i].gainArray_[k] = kSpread;
    }  // end of k loop
  }    // end of i loop
}

}  // namespace ocs2
