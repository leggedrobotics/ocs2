/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <iostream>

#include <ocs2_core/control/FeedforwardController.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FeedforwardController::FeedforwardController(const scalar_array_t& controllerTime, const vector_array_t& stateTrajectory,
                                             ControllerBase* controller) {
  if (controllerTime.size() != stateTrajectory.size()) {
    throw std::runtime_error("FeedforwardController Constructor: controllerTime and stateTrajectory sizes mismatch.");
  }

  timeStamp_ = controllerTime;
  uffArray_.clear();
  uffArray_.reserve(controllerTime.size());

  auto iTime = controllerTime.cbegin();
  auto iState = stateTrajectory.cbegin();

  for (; iTime != controllerTime.end() and iState != stateTrajectory.end(); ++iTime, ++iState) {
    uffArray_.emplace_back(controller->computeInput(*iTime, *iState));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FeedforwardController::FeedforwardController(const FeedforwardController& other)
    : FeedforwardController(other.timeStamp_, other.uffArray_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FeedforwardController::FeedforwardController(FeedforwardController&& other) : ControllerBase(other) {
  swap(other, *this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FeedforwardController& FeedforwardController::operator=(FeedforwardController rhs) {
  swap(rhs, *this);
  return *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FeedforwardController::setController(const scalar_array_t& controllerTime, const vector_array_t& controllerFeedforward) {
  timeStamp_ = controllerTime;
  uffArray_ = controllerFeedforward;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t FeedforwardController::computeInput(scalar_t t, const vector_t& x) {
  return LinearInterpolation::interpolate(t, timeStamp_, uffArray_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FeedforwardController::flatten(const scalar_array_t& timeArray, const std::vector<std::vector<float>*>& flatArray2) const {
  const auto timeSize = timeArray.size();
  const auto dataSize = flatArray2.size();

  if (timeSize != dataSize) {
    throw std::runtime_error("timeSize and dataSize must be equal in flatten method.");
  }

  for (size_t i = 0; i < timeSize; i++) {
    flattenSingle(timeArray[i], *(flatArray2[i]));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FeedforwardController::flattenSingle(scalar_t time, std::vector<float>& flatArray) const {
  /* Serialized feedforward controller:
   * data = [
   *   [ uff(t0)[:] ],
   *   [ uff(t1)[:] ],
   *   ...
   *   [ uff(tN)[:] ]
   * ]
   */

  const vector_t uff = LinearInterpolation::interpolate(time, timeStamp_, uffArray_);

  flatArray = std::vector<float>(uff.data(), uff.data() + uff.rows());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FeedforwardController FeedforwardController::unFlatten(const scalar_array_t& timeArray,
                                                       const std::vector<std::vector<float> const*>& flatArray2) {
  vector_array_t uffArray;
  uffArray.reserve(flatArray2.size());

  for (const auto& arr : flatArray2) {  // loop through time
    uffArray.emplace_back(Eigen::Map<const Eigen::VectorXf>(arr->data(), arr->size()).cast<scalar_t>());
  }
  return FeedforwardController(timeArray, std::move(uffArray));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FeedforwardController::concatenate(const ControllerBase* nextController, int index, int length) {
  if (auto nextFfwdCtrl = dynamic_cast<const FeedforwardController*>(nextController)) {
    if (!timeStamp_.empty() && timeStamp_.back() > nextFfwdCtrl->timeStamp_.front()) {
      throw std::runtime_error("Concatenate requires that the nextController comes later in time.");
    }
    int last = index + length;
    timeStamp_.insert(timeStamp_.end(), nextFfwdCtrl->timeStamp_.begin() + index, nextFfwdCtrl->timeStamp_.begin() + last);
    uffArray_.insert(uffArray_.end(), nextFfwdCtrl->uffArray_.begin() + index, nextFfwdCtrl->uffArray_.begin() + last);
  } else {
    throw std::runtime_error("Concatenate only works with controllers of the same type.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
int FeedforwardController::size() const {
  return timeStamp_.size();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ControllerType FeedforwardController::getType() const {
  return ControllerType::FEEDFORWARD;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FeedforwardController::clear() {
  timeStamp_.clear();
  uffArray_.clear();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool FeedforwardController::empty() const {
  return timeStamp_.empty();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FeedforwardController* FeedforwardController::clone() const {
  return new FeedforwardController(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FeedforwardController::display() const {
  std::cerr << *this << '\n';
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void swap(FeedforwardController& a, FeedforwardController& b) noexcept {
  std::swap(a.timeStamp_, b.timeStamp_);
  std::swap(a.uffArray_, b.uffArray_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::ostream& operator<<(std::ostream& out, const FeedforwardController& controller) {
  for (int i = 0; i < controller.timeStamp_.size(); i++) {
    out << "t\t" << controller.timeStamp_[i] << "\tu\t" << controller.uffArray_[i].transpose() << '\n';
  }
  return out;
}

}  // namespace ocs2
